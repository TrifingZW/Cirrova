#include "Components/CirrovaControlComponent.h"

#include <EnhancedInputComponent.h>
#include <EnhancedInputSubsystems.h>
#include <InputMappingContext.h>
#include <Components/SingularisMorphVehicleSimulationComponent.h>
#include <GameFramework/PlayerController.h>
#include <GameFramework/SpringArmComponent.h>
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
	static const ConstructorHelpers::FObjectFinder<UInputAction> LookInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_Look.IA_Look")
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
	static const ConstructorHelpers::FObjectFinder<UInputAction> ReverseInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_Reverse.IA_Reverse")
	);
	static const ConstructorHelpers::FObjectFinder<UInputAction> ChangeUpInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_ChangeUp.IA_ChangeUp")
	);
	static const ConstructorHelpers::FObjectFinder<UInputAction> ChangeDownInputActionFinder(
		TEXT("/Cirrova/Inputs/Actions/IA_ChangeDown.IA_ChangeDown")
	);

	if (InputMappingContextFinder.Succeeded())
		InputMappingContext = InputMappingContextFinder.Object;
	if (LookInputActionFinder.Succeeded())
		LookInputAction = LookInputActionFinder.Object;
	if (SteeringInputActionFinder.Succeeded())
		SteeringInputAction = SteeringInputActionFinder.Object;
	if (ThrottleInputActionFinder.Succeeded())
		ThrottleInputAction = ThrottleInputActionFinder.Object;
	if (BrakeInputActionFinder.Succeeded())
		BrakeInputAction = BrakeInputActionFinder.Object;
	if (HandbrakeInputActionFinder.Succeeded())
		HandbrakeInputAction = HandbrakeInputActionFinder.Object;
	if (ReverseInputActionFinder.Succeeded())
		ReverseInputAction = ReverseInputActionFinder.Object;
	if (ChangeUpInputActionFinder.Succeeded())
		ChangeUpInputAction = ChangeUpInputActionFinder.Object;
	if (ChangeDownInputActionFinder.Succeeded())
		ChangeDownInputAction = ChangeDownInputActionFinder.Object;
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

void UCirrovaControlComponent::Enable()
{
	if (bIsEnable) return;

	SetEnabled(true);
}

void UCirrovaControlComponent::Disable()
{
	if (!bIsEnable) return;

	SetEnabled(false);
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
	// 同步本地控制声明：启用网络物理预测时，未标记本地控制的载具，其控制输入会在
	// 物理线程被网络输入覆盖而完全失效（表现为载具无法控制）
	if (OldVehicle)
	{
		if (USingularisMorphVehicleSimulationComponent* OldSimulation =
			OldVehicle->FindComponentByClass<USingularisMorphVehicleSimulationComponent>())
			OldSimulation->SetLocallyControlled(false);
	}

	if (const AActor* Vehicle = ControlledVehicle.Get())
	{
		if (USingularisMorphVehicleSimulationComponent* Simulation =
			Vehicle->FindComponentByClass<USingularisMorphVehicleSimulationComponent>())
			Simulation->SetLocallyControlled(true);
	}

	// 输入映射上下文随受控状态注册或移除
	RefreshInput();
}

void UCirrovaControlComponent::SetEnabled(bool bEnable)
{
	// 1) 本地玩家检查
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;

	// 2) 幂等性检查，若状态未变更则直接返回
	if (bIsEnable == bEnable) return;

	// 3) 捕获旧状态后写入新状态
	const bool bOldEnable = bIsEnable;
	bIsEnable = bEnable;

	// 4) 响应式编程：应用副作用
	ApplyEnabled(bOldEnable);
}

void UCirrovaControlComponent::ApplyEnabled(bool bOldEnable) const
{
	RefreshInput();
}

// ReSharper disable CppMemberFunctionMayBeConst

void UCirrovaControlComponent::HandleLookInputAction(const FInputActionValue& InputActionValue)
{
	// 1) 解析受控载具的相机臂
	USpringArmComponent* CameraBoom = FindCameraBoom();
	if (!IsValid(CameraBoom)) return;

	// 2) 读取二维视角输入：X 为偏航、Y 为俯仰（灵敏度等缩放由 EnhancedInput 映射修饰器负责）
	const FVector2D LookDelta = InputActionValue.Get<FVector2D>();

	// 3) 仅写局部旋转：世界旋转由相机臂从载具继承，从而视角随载具俯仰/翻滚保持自由；
	//    俯仰夹紧避免翻转，偏航不限制，滚转保持归零
	const FRotator Current = CameraBoom->GetRelativeRotation();
	CameraBoom->SetRelativeRotation(
		FRotator(
			FMath::Clamp(Current.Pitch + LookDelta.Y, PitchMin, PitchMax),
			Current.Yaw + LookDelta.X,
			0.0
		)
	);
}

void UCirrovaControlComponent::HandleSteering(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::SteeringControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleThrottle(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::ThrottleControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleBrakeTriggered(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::BrakeControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleBrakeStarted(const FInputActionValue& InputActionValue)
{
	HandleBrakeTriggered(InputActionValue);
}

void UCirrovaControlComponent::HandleBrakeCompleted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::BrakeControlName, 0.0);
}

void UCirrovaControlComponent::HandleHandbrakeStarted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::HandbrakeControlName, 1.0);
}

void UCirrovaControlComponent::HandleHandbrakeTriggered(const FInputActionValue& InputActionValue)
{
	// 输入生产者每物理步整体覆盖：逐帧重写才能让手刹在按住期间保持
	HandleHandbrakeStarted(InputActionValue);
}

void UCirrovaControlComponent::HandleHandbrakeCompleted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputAxis1D(Chaos::HandbrakeControlName, 0.0);
}

void UCirrovaControlComponent::HandleReverseStarted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	// 倒挡控制是布尔通道：按住期间物理线程的自动倒挡逻辑将目标挡位拉入倒挡
	VehicleSimulation->SetInputBool(Chaos::ReverseControlName, true);
}

void UCirrovaControlComponent::HandleReverseTriggered(const FInputActionValue& InputActionValue)
{
	// 输入生产者每物理步整体覆盖：逐帧重写才能让倒挡在按住期间保持
	HandleReverseStarted(InputActionValue);
}

void UCirrovaControlComponent::HandleReverseCompleted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputBool(Chaos::ReverseControlName, false);
}

void UCirrovaControlComponent::HandleChangeUpStarted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	// 换挡输入为消费后自动清除的脉冲通道，单次按下恰好产生一次换挡
	VehicleSimulation->SetInputBool(Chaos::ChangeUpControlName, true);
}

void UCirrovaControlComponent::HandleChangeDownStarted(const FInputActionValue& InputActionValue)
{
	const USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	VehicleSimulation->SetInputBool(Chaos::ChangeDownControlName, true);
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
	// Looking
	EnhancedInputComponent->BindAction(
		LookInputAction,
		ETriggerEvent::Triggered,
		this,
		&UCirrovaControlComponent::HandleLookInputAction
	);

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
		&UCirrovaControlComponent::HandleBrakeTriggered
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

	// Handbrake：Triggered 在按住期间逐帧维持通道（事件即写入时机，无组件 Tick）
	EnhancedInputComponent->BindAction(
		HandbrakeInputAction,
		ETriggerEvent::Started,
		this,
		&UCirrovaControlComponent::HandleHandbrakeStarted
	);
	EnhancedInputComponent->BindAction(
		HandbrakeInputAction,
		ETriggerEvent::Triggered,
		this,
		&UCirrovaControlComponent::HandleHandbrakeTriggered
	);
	EnhancedInputComponent->BindAction(
		HandbrakeInputAction,
		ETriggerEvent::Completed,
		this,
		&UCirrovaControlComponent::HandleHandbrakeCompleted
	);

	// 5) 挡位输入为可选动作：资产缺失时仅跳过对应绑定，不影响基础操控；
	//    倒挡键与手刹按住期间由 Triggered 逐帧重写（输入生产者逐物理步整体覆盖）；
	//    升/降挡脉冲由手动挡变速箱消费（消费即清零，单次按下恰好一次换挡）
	if (IsValid(ReverseInputAction))
	{
		EnhancedInputComponent->BindAction(
			ReverseInputAction,
			ETriggerEvent::Started,
			this,
			&UCirrovaControlComponent::HandleReverseStarted
		);
		EnhancedInputComponent->BindAction(
			ReverseInputAction,
			ETriggerEvent::Triggered,
			this,
			&UCirrovaControlComponent::HandleReverseTriggered
		);
		EnhancedInputComponent->BindAction(
			ReverseInputAction,
			ETriggerEvent::Completed,
			this,
			&UCirrovaControlComponent::HandleReverseCompleted
		);
	}

	if (IsValid(ChangeUpInputAction))
	{
		EnhancedInputComponent->BindAction(
			ChangeUpInputAction,
			ETriggerEvent::Started,
			this,
			&UCirrovaControlComponent::HandleChangeUpStarted
		);
	}

	if (IsValid(ChangeDownInputAction))
	{
		EnhancedInputComponent->BindAction(
			ChangeDownInputAction,
			ETriggerEvent::Started,
			this,
			&UCirrovaControlComponent::HandleChangeDownStarted
		);
	}
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

	// 4) 依据开启与受控状态注册或移除映射上下文
	if (bIsEnable && ControlledVehicle.IsValid())
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

USpringArmComponent* UCirrovaControlComponent::FindCameraBoom() const
{
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return nullptr;
	if (!ControlledVehicle.IsValid()) return nullptr;

	return ControlledVehicle->FindComponentByClass<USpringArmComponent>();
}
