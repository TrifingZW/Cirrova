#pragma once

#include <CoreMinimal.h>
#include <InputActionValue.h>
#include <Components/ActorComponent.h>

#include "CirrovaControlComponent.generated.h"

class USingularisMorphVehicleSimulationComponent;
class UInputAction;
class UInputMappingContext;

UCLASS(
	Blueprintable,
	BlueprintType,
	ClassGroup = ("Cirrova"),
	meta = (BlueprintSpawnableComponent, DisplayName = "虹涡控制组件")
)
class CIRROVA_API UCirrovaControlComponent : public UActorComponent
{
	GENERATED_BODY()

public:
#pragma region Parameter

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "输入优先级")
	)
	int32 InputPriority = 10;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "输入映射上下文")
	)
	UInputMappingContext* InputMappingContext = nullptr;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "转向输入动作")
	)
	UInputAction* SteeringInputAction = nullptr;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "油门输入动作")
	)
	UInputAction* ThrottleInputAction = nullptr;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "制动输入动作")
	)
	UInputAction* BrakeInputAction = nullptr;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "手刹输入动作")
	)
	UInputAction* HandbrakeInputAction = nullptr;

#pragma endregion

private:
#pragma region Internal Variable

	TWeakObjectPtr<APlayerController> OwnerPlayerController = nullptr;
	TWeakObjectPtr<AActor> ControlledVehicle = nullptr;

#pragma endregion

public:
#pragma region Constructors

	UCirrovaControlComponent();

#pragma endregion

#pragma region ActorComponent Interface

	virtual void BeginPlay() override;

#pragma endregion

#pragma region API

	UFUNCTION(
		BlueprintCallable,
		Category = "虹涡控制组件",
		meta = (DisplayName = "SetControlled")
	)
	void SetControlled(AActor* Vehicle);

#pragma endregion

private:
#pragma region Callback

	void HandleSteering(const FInputActionValue& InputActionValue);
	void HandleThrottle(const FInputActionValue& InputActionValue);
	void HandleBrakTriggered(const FInputActionValue& InputActionValue);
	void HandleBrakeStarted(const FInputActionValue& InputActionValue);
	void HandleBrakeCompleted(const FInputActionValue& InputActionValue);
	void HandleHandbrakeStarted(const FInputActionValue& InputActionValue);
	void HandleHandbrakeCompleted(const FInputActionValue& InputActionValue);

#pragma endregion

#pragma region Internal Function

	void BindInput();

	void RefreshInput() const;
	
	/** 解析当前受控载具上的变型载具仿真组件（前置校验失败时返回 nullptr） */
	USingularisMorphVehicleSimulationComponent* FindVehicleSimulation() const;

#pragma endregion
};
