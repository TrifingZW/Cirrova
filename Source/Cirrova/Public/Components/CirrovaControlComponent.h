#pragma once

#include <CoreMinimal.h>
#include <InputActionValue.h>
#include <Components/ActorComponent.h>

#include "CirrovaControlComponent.generated.h"

class UInputAction;
class UInputMappingContext;
class USingularisMorphVehicleSimulationComponent;
class USpringArmComponent;

/**
 * 虹涡控制组件。
 *
 * 本地玩家控制设计模式典范：
 * 挂载于本地 PlayerController，经 EnhancedInput 绑定输入动作并将输入值直写
 * 受控载具的变型载具仿真组件；受控状态切换时同步增删输入映射上下文，全程本地生效、无复制。
 */
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

	/** 输入映射上下文的注册优先级，数值越大优先级越高 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "输入优先级")
	)
	int32 InputPriority = 10;

	/** 载具操控输入映射上下文，受控时注册、释放时移除 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "输入映射上下文")
	)
	UInputMappingContext* InputMappingContext = nullptr;

	/** 视角输入动作 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",

		meta = (DisplayName = "视角输入动作")
	)
	UInputAction* LookInputAction = nullptr;

	/** 转向输入动作，写入 Chaos 转向控制轴 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "转向输入动作")
	)
	UInputAction* SteeringInputAction = nullptr;

	/** 油门输入动作，触发时直写 Chaos 油门控制轴 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "油门输入动作")
	)
	UInputAction* ThrottleInputAction = nullptr;

	/** 制动输入动作，触发时直写 Chaos 制动控制轴 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "制动输入动作")
	)
	UInputAction* BrakeInputAction = nullptr;

	/** 手刹输入动作，按下写入 1、抬起写入 0（按住期间逐帧重写） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "手刹输入动作")
	)
	UInputAction* HandbrakeInputAction = nullptr;

	/** 倒挡输入动作，按下写入倒挡控制、抬起归零（按住期间逐帧重写） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "倒挡输入动作")
	)
	UInputAction* ReverseInputAction = nullptr;

	/** 升挡输入动作，按下写入一次升挡脉冲（手动挡换挡；自动挡变速箱不消费该输入） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "升挡输入动作")
	)
	UInputAction* ChangeUpInputAction = nullptr;

	/** 降挡输入动作，按下写入一次降挡脉冲（手动挡换挡，连续降挡可经空挡降至倒挡） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "降挡输入动作")
	)
	UInputAction* ChangeDownInputAction = nullptr;

	/** 相机臂局部俯仰下限（度） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "俯仰下限")
	)
	float PitchMin = -70.0f;

	/** 相机臂局部俯仰上限（度） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "俯仰上限")
	)
	float PitchMax = 70.0f;

#pragma endregion

private:
#pragma region State

	/** Owner 本地玩家控制器，BeginPlay 时解析填充 */
	TWeakObjectPtr<APlayerController> OwnerPlayerController = nullptr;

	/** 当前受控载具，仅本地控制器可变更 */
	TWeakObjectPtr<AActor> ControlledVehicle = nullptr;

	/** 组件是否已开启 */
	bool bIsEnable = true;

#pragma endregion

public:
#pragma region Constructors

	/** 默认构造函数，关闭复制与 Tick 并加载插件内置输入资产 */
	UCirrovaControlComponent();

#pragma endregion

#pragma region ActorComponent Interface

	/** 校验 Owner 为 PlayerController，缓存本地控制器并绑定输入动作 */
	virtual void BeginPlay() override;

#pragma endregion

#pragma region API

	/**
	 * 是否正在控制载具。
	 *
	 * @return 存在有效受控载具时返回 true。
	 */
	UFUNCTION(
		BlueprintPure,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "Controlled")
	)
	bool Controlled() const { return ControlledVehicle.IsValid(); }

	/**
	 * 获取当前受控载具。
	 *
	 * @return 受控载具，未控制时返回 nullptr。
	 */
	UFUNCTION(
		BlueprintPure,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "GetVehicle")
	)
	AActor* GetVehicle() const { return ControlledVehicle.Get(); }

	/**
	 * 组件是否已开启。
	 *
	 * @return 已开启时返回 true。
	 */
	UFUNCTION(
		BlueprintPure,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "Enabled")
	)
	bool Enabled() const { return bIsEnable; }

	/**
	 * 开始控制指定载具。
	 *
	 * 本地控制器操作。注册输入映射上下文，输入值直写载具的变型载具仿真组件，
	 * 已在控制中或入参非法时静默忽略。
	 *
	 * @param Vehicle 要控制的载具 Actor。
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "控制")
	)
	void Control(AActor* Vehicle);

	/**
	 * 释放当前受控载具。
	 *
	 * 本地控制器操作。移除输入映射上下文并清空受控状态，未控制时静默忽略。
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "释放")
	)
	void Release();

	/**
	 * 开启组件。
	 *
	 * 本地控制器操作。恢复输入处理，已开启时静默忽略。
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "开启")
	)
	void Enable();

	/**
	 * 关闭组件。
	 *
	 * 本地控制器操作。停止处理输入而不清除受控状态，已关闭时静默忽略。
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "虹涡控制组件|API",
		meta = (DisplayName = "关闭")
	)
	void Disable();

#pragma endregion

private:
#pragma region Callback

	/** 视角输入触发时在载具本地系内驱动相机臂俯仰与偏航，俯仰按设定范围夹紧 */
	void HandleLookInputAction(const FInputActionValue& InputActionValue);

	/** 转向输入触发时写入 Chaos 转向控制轴（按住期间逐帧触发） */
	void HandleSteering(const FInputActionValue& InputActionValue);

	/** 油门输入触发时写入 Chaos 油门控制轴（按住期间逐帧触发） */
	void HandleThrottle(const FInputActionValue& InputActionValue);

	/** 制动输入触发时写入 Chaos 制动控制轴（按住期间逐帧触发） */
	void HandleBrakeTriggered(const FInputActionValue& InputActionValue);

	/** 制动输入按下时写入 Chaos 制动控制轴，保证瞬时按键生效 */
	void HandleBrakeStarted(const FInputActionValue& InputActionValue);

	/** 制动输入抬起时制动控制轴归零 */
	void HandleBrakeCompleted(const FInputActionValue& InputActionValue);

	/** 手刹输入按下时置 1 */
	void HandleHandbrakeStarted(const FInputActionValue& InputActionValue);

	/** 手刹输入按住期间逐帧置 1，维持按住状态 */
	void HandleHandbrakeTriggered(const FInputActionValue& InputActionValue);

	/** 手刹输入抬起时归零 */
	void HandleHandbrakeCompleted(const FInputActionValue& InputActionValue);

	/** 倒挡键按下时置倒挡控制为真 */
	void HandleReverseStarted(const FInputActionValue& InputActionValue);

	/** 倒挡键按住期间逐帧置真，维持按住状态 */
	void HandleReverseTriggered(const FInputActionValue& InputActionValue);

	/** 倒挡键抬起时倒挡控制归零 */
	void HandleReverseCompleted(const FInputActionValue& InputActionValue);

	/** 升挡输入按下时写入一次升挡脉冲（消费即清零的脉冲通道） */
	void HandleChangeUpStarted(const FInputActionValue& InputActionValue);

	/** 降挡输入按下时写入一次降挡脉冲（消费即清零的脉冲通道） */
	void HandleChangeDownStarted(const FInputActionValue& InputActionValue);

#pragma endregion

#pragma region Internal Function

	/** 将输入动作绑定到 Owner 的 EnhancedInputComponent；输入资产缺失或 Owner 非本地控制器时放弃绑定 */
	void BindInput();

	/** 依据受控与开启状态注册或移除输入映射上下文 */
	void RefreshInput() const;

	/**
	 * 设置受控载具并应用副作用。
	 *
	 * 本地控制器，幂等。先快照旧值，再赋值新值，最后调用 ApplyVehicle。
	 *
	 * @param Vehicle 新的受控载具，传入 nullptr 表示释放。
	 */
	void SetVehicle(AActor* Vehicle);

	/**
	 * 应用受控载具变化到输入映射。
	 *
	 * @param OldVehicle 变化前的旧受控载具，当前实现未使用。
	 */
	void ApplyVehicle(const AActor* OldVehicle) const;

	/**
	 * 设置开启状态并应用副作用。
	 *
	 * 本地控制器，幂等。先快照旧值，再赋值新值，最后调用 ApplyEnabled。
	 *
	 * @param bEnable 新的开启状态。
	 */
	void SetEnabled(bool bEnable);

	/**
	 * 应用开启状态变化到输入映射。
	 *
	 * @param bOldEnable 变化前的旧开启状态。
	 */
	void ApplyEnabled(bool bOldEnable) const;

	/**
	 * 解析当前受控载具上的变型载具仿真组件。
	 *
	 * @return 受控载具上的仿真组件，非本地控制器或无受控载具时返回 nullptr。
	 */
	USingularisMorphVehicleSimulationComponent* FindVehicleSimulation() const;

	/**
	 * 解析当前受控载具上的相机臂组件。
	 *
	 * @return 受控载具上的弹簧臂，非本地控制器或无受控载具时返回 nullptr。
	 */
	USpringArmComponent* FindCameraBoom() const;

#pragma endregion
};
