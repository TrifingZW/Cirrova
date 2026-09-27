#include "Components/CirrovaControlComponent.h"

#include <Components/SingularisMorphVehicleSimulationComponent.h>
#include <EnhancedInputComponent.h>
#include <EnhancedInputSubsystems.h>
#include <GameFramework/PlayerController.h>
#include <InputMappingContext.h>
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

	checkf(
		GetOwner()->IsA<APlayerController>(),
		TEXT("CirrovaControlComponent: Owner is not PlayerController")
	);

	OwnerPlayerController = Cast<APlayerController>(GetOwner());

	BindInput();
}

void UCirrovaControlComponent::SetControlled(AActor* Vehicle)
{
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;

	if (ControlledVehicle == Vehicle) return;
	ControlledVehicle = Vehicle;

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
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;

	// 任一输入动作资产缺失时放弃整体绑定，避免运行期逐条告警
	if (!IsValid(SteeringInputAction) || !IsValid(ThrottleInputAction) ||
		!IsValid(BrakeInputAction) || !IsValid(HandbrakeInputAction))
		return;

	// 绑定输入动作
	UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(
		OwnerPlayerController->InputComponent
	);
	if (!IsValid(EnhancedInputComponent)) return;

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
	if (!OwnerPlayerController.IsValid() || !OwnerPlayerController->IsLocalController()) return;
	if (!IsValid(InputMappingContext)) return;

	UEnhancedInputLocalPlayerSubsystem* Subsystem =
		ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(OwnerPlayerController->GetLocalPlayer());
	if (!IsValid(Subsystem)) return;

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
