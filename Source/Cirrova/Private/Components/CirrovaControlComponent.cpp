#include "Components/CirrovaControlComponent.h"

#include <EnhancedInputComponent.h>
#include <EnhancedInputSubsystems.h>
#include <InputMappingContext.h>
#include <Components/SingularisMorphVehicleSimulationComponent.h>
#include <GameFramework/PlayerController.h>
#include <UObject/ConstructorHelpers.h>

UCirrovaControlComponent::UCirrovaControlComponent()
{
	SetIsReplicatedByDefault(false);

	PrimaryComponentTick.bStartWithTickEnabled = false;
	PrimaryComponentTick.bCanEverTick = false;

	bAutoActivate = true;

	static const ConstructorHelpers::FObjectFinder<UInputMappingContext> InputMappingContextFinder(
		TEXT("/Cirrova/Inputs/IMC_Cirrova_Vehicle.IMC_Cirrova_Vehicle")
	);
	static const ConstructorHelpers::FObjectFinder<UInputAction> SteeringInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_Steering.IA_Steering")
	);
	static const ConstructorHelpers::FObjectFinder<UInputAction> ThrottleInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_Throttle.IA_Throttle")
	);
	static const ConstructorHelpers::FObjectFinder<UInputAction> BrakeInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_Brake.IA_Brake")
	);
	static const ConstructorHelpers::FObjectFinder<UInputAction> HandbrakeInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_Handbrake.IA_Handbrake")
	);

	InputMappingContext = InputMappingContextFinder.Object;
	SteeringInputAction = SteeringInputActionFinder.Object;
	ThrottleInputAction = ThrottleInputActionFinder.Object;
	BrakeInputAction = BrakeInputActionFinder.Object;
	HandbrakeInputAction = HandbrakeInputActionFinder.Object;
}

void UCirrovaControlComponent::BeginPlay()
{
	Super::BeginPlay();

	// 1) 契约校验：组件必须挂载于 PlayerController
	checkf(
		GetOwner()->IsA<APlayerController>(),
		TEXT("CirrovaControlComponent: Owner is not PlayerController")
	);

	// 2) 缓存本地控制器
	OwnerPlayerController = Cast<APlayerController>(GetOwner());

	// 3) 绑定输入动作
	BindInput();
}

void UCirrovaControlComponent::Control(AActor* Vehicle)
{
	if (Controlled() || !IsValid(Vehicle)) return;

	SetVehicle(Vehicle);
}

void UCirrovaControlComponent::Release()
{
	if (!Controlled()) return;

	SetVehicle(nullptr);
}

void UCirrovaControlComponent::SetVehicle(AActor* Vehicle)
{
	// 1) 本地玩家检查
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;

	// 2) 幂等性检查，若状态未变更则直接返回
	if (ControlledVehicle == Vehicle) return;

	// 3) 捕获旧状态后写入新状态
	const AActor* OldVehicle = ControlledVehicle.Get();
	ControlledVehicle = Vehicle;

	// 4) 响应式编程：应用副作用
	ApplyVehicle(OldVehicle);
}

void UCirrovaControlComponent::ApplyVehicle(const AActor* OldVehicle) const
{
	// 输入映射上下文随受控状态注册或移除
	RefreshInput();
}

// ReSharper disable CppMemberFunctionMayBeConst

void UCirrovaControlComponent::HandleSteering(const FInputActionValue& InputActionValue)
{
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::SteeringControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleThrottle(const FInputActionValue& InputActionValue)
{
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::ThrottleControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleBrakTriggered(const FInputActionValue& InputActionValue)
{
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::BrakeControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleBrakeStarted(const FInputActionValue& InputActionValue)
{
	HandleBrakTriggered(InputActionValue);
}

void UCirrovaControlComponent::HandleBrakeCompleted(const FInputActionValue& InputActionValue)
{
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::BrakeControlName, 0.0);
}

void UCirrovaControlComponent::HandleHandbrakeStarted(const FInputActionValue& InputActionValue)
{
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::HandbrakeControlName, 1.0);
}

void UCirrovaControlComponent::HandleHandbrakeCompleted(const FInputActionValue& InputActionValue)
{
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::HandbrakeControlName, 0.0);
}

// ReSharper restore CppMemberFunctionMayBeConst

void UCirrovaControlComponent::BindInput()
{
	// 1) 卫语句：仅本地控制器可绑定输入
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;

	// 2) 卫语句：任一输入动作资产缺失时放弃整体绑定，避免运行期逐条告警
	if (!IsValid(SteeringInputAction) || !IsValid(ThrottleInputAction) ||
		!IsValid(BrakeInputAction) || !IsValid(HandbrakeInputAction))
		return;

	// 3) 卫语句：Owner 未启用 EnhancedInput 时无法绑定
	UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(
		OwnerPlayerController->InputComponent
	);
	if (!IsValid(EnhancedInputComponent)) return;

	// 4) 逐动作绑定回调
	// Steering
	EnhancedInputComponent->BindAction(
		SteeringInputAction,
		ETriggerEvent::Triggered,
		this,
		&UCirrovaControlComponent::HandleSteering
	);

	// Throttle
	EnhancedInputComponent->BindAction(
		ThrottleInputAction,
		ETriggerEvent::Triggered,
		this,
		&UCirrovaControlComponent::HandleThrottle
	);

	// Brake
	EnhancedInputComponent->BindAction(
		BrakeInputAction,
		ETriggerEvent::Triggered,
		this,
		&UCirrovaControlComponent::HandleBrakTriggered
	);
	EnhancedInputComponent->BindAction(
		BrakeInputAction,
		ETriggerEvent::Started,
		this,
		&UCirrovaControlComponent::HandleBrakeStarted
	);
	EnhancedInputComponent->BindAction(
		BrakeInputAction,
		ETriggerEvent::Completed,
		this,
		&UCirrovaControlComponent::HandleBrakeCompleted
	);

	// Handbrake
	EnhancedInputComponent->BindAction(
		HandbrakeInputAction,
		ETriggerEvent::Started,
		this,
		&UCirrovaControlComponent::HandleHandbrakeStarted
	);
	EnhancedInputComponent->BindAction(
		HandbrakeInputAction,
		ETriggerEvent::Completed,
		this,
		&UCirrovaControlComponent::HandleHandbrakeCompleted
	);
}

void UCirrovaControlComponent::RefreshInput() const
{
	// 1) 卫语句：仅本地控制器持有 EnhancedInput 子系统
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;

	// 2) 卫语句：未配置映射上下文时无需处理
	if (!IsValid(InputMappingContext)) return;

	// 3) 获取本地玩家的 EnhancedInput 子系统
	UEnhancedInputLocalPlayerSubsystem* Subsystem =
		ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(OwnerPlayerController->GetLocalPlayer());
	if (!IsValid(Subsystem)) return;

	// 4) 依据受控状态注册或移除映射上下文
	if (ControlledVehicle.IsValid())
		Subsystem->AddMappingContext(InputMappingContext, InputPriority);
	else
		Subsystem->RemoveMappingContext(InputMappingContext);
}

USingularisMorphVehicleSimulationComponent* UCirrovaControlComponent::FindVehicleSimulation() const
{
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return nullptr;
	if (!ControlledVehicle.IsValid()) return nullptr;

	return ControlledVehicle->FindComponentByClass<USingularisMorphVehicleSimulationComponent>();
}
