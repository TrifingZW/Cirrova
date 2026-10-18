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
 * 挂载于本地 PlayerController，经 EnhancedInput 缓存原始输入；驱动通道（油门/制动/
 * 手刹/倒挡）由组件 Tick 按驾驶规则仲裁后写入受控载具的变型载具仿真组件，
 * 转向/视角直写；受控状态切换时同步增删输入映射上下文并启停仲裁 Tick，
 * 全程本地生效、无复制。
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

	/** 油门输入动作，触发时缓存原始油门值，写入由驱动仲裁统一完成 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "油门输入动作")
	)
	UInputAction* ThrottleInputAction = nullptr;

	/** 制动输入动作，触发时缓存原始制动值，写入由驱动仲裁统一完成 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "制动输入动作")
	)
	UInputAction* BrakeInputAction = nullptr;

	/** 手刹输入动作，按下缓存 1、抬起缓存 0，写入由驱动仲裁统一完成 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "手刹输入动作")
	)
	UInputAction* HandbrakeInputAction = nullptr;

	/** 倒挡输入动作，按下缓存按住状态；街机倒挡下经仲裁与刹车派生合并生效 */
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

	/**
	 * 街机倒挡（生效前提：受控载具的变速箱开启 AutoReverse，默认开启）。
	 *
	 * 开启后：按住 S 在低速时自动切入倒挡，S 转为倒挡油门（单键完成刹停→倒车，
	 * 无需按 W）；倒挡中按 W 优先切回前进挡。关闭后全部驱动输入直通。
	 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "街机倒挡")
	)
	bool bReverseAsBrake = true;

	/** 倒车中按 W 是否转为全刹车直至低于方向切换速度（需开启街机倒挡） */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "油门即刹车", EditCondition = "bReverseAsBrake")
	)
	bool bThrottleAsBrake = true;

	/**
	 * 方向切换速度阈值（km/h）。
	 *
	 * 车速绝对值低于该值时按住 S 才会切入倒挡；倒车中按 W 作刹车，
	 * 车速降到该值以下后 W 恢复为油门。默认值取自经典载具插件（100 cm/s）。
	 */
	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "虹涡控制组件",
		meta = (DisplayName = "方向切换速度", ClampMin = "0.0", UIMin = "0.0")
	)
	float DirectionSwitchSpeed = 3.6f;

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

	/** 原始油门输入缓存（EnhancedInput 事件写入，驱动仲裁消费） */
	float RawThrottleInput = 0.0f;

	/** 原始制动输入缓存 */
	float RawBrakeInput = 0.0f;

	/** 原始手刹输入缓存 */
	float RawHandbrakeInput = 0.0f;

	/** 独立倒挡键按住状态缓存 */
	bool bRawReverseKeyHeld = false;

	/**
	 * 街机倒挡模式。
	 *
	 * 进入后 S 语义为倒挡油门并持续写入倒挡控制；W 按下即退出。
	 * 换车时随驱动仲裁状态一并重置。
	 */
	bool bReverseMode = false;

#pragma endregion

public:
#pragma region Constructors

	/** 默认构造函数，关闭复制、仲裁 Tick 按需启停，并加载插件内置输入资产 */
	UCirrovaControlComponent();

#pragma endregion

#pragma region ActorComponent Interface

	/** 校验 Owner 为 PlayerController，缓存本地控制器并绑定输入动作 */
	virtual void BeginPlay() override;

	/** 将缓存的原始输入按驾驶规则仲裁后写入仿真组件（受控期间启用） */
	virtual void TickComponent(
		float DeltaTime,
		ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;

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
	 * 本地控制器操作。注册输入映射上下文并开启驱动仲裁，
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

	/** 转向输入触发时写入 Chaos 转向控制轴（直写，不经驱动仲裁） */
	void HandleSteering(const FInputActionValue& InputActionValue);

	/** 油门输入触发时缓存原始油门值，写入由驱动仲裁统一完成 */
	void HandleThrottle(const FInputActionValue& InputActionValue);

	/** 油门输入抬起时清零原始油门缓存 */
	void HandleThrottleCompleted(const FInputActionValue& InputActionValue);

	/** 制动输入触发时缓存原始制动值，写入由驱动仲裁统一完成 */
	void HandleBrakeTriggered(const FInputActionValue& InputActionValue);

	/** 制动输入按下时缓存原始制动值，保证瞬时按键生效 */
	void HandleBrakeStarted(const FInputActionValue& InputActionValue);

	/** 制动输入抬起时清零原始制动缓存 */
	void HandleBrakeCompleted(const FInputActionValue& InputActionValue);

	/** 手刹输入按下时缓存置 1，写入由驱动仲裁统一完成 */
	void HandleHandbrakeStarted(const FInputActionValue& InputActionValue);

	/** 手刹输入抬起时清零原始手刹缓存 */
	void HandleHandbrakeCompleted(const FInputActionValue& InputActionValue);

	/** 倒挡键按下时缓存按住状态，写入由驱动仲裁统一完成 */
	void HandleReverseStarted(const FInputActionValue& InputActionValue);

	/** 倒挡键抬起时清零按住状态缓存 */
	void HandleReverseCompleted(const FInputActionValue& InputActionValue);

	/** 升挡输入按下时写入一次升挡脉冲（消费即清零的脉冲通道，直写不经仲裁） */
	void HandleChangeUpStarted(const FInputActionValue& InputActionValue);

	/** 降挡输入按下时写入一次降挡脉冲（消费即清零的脉冲通道，直写不经仲裁） */
	void HandleChangeDownStarted(const FInputActionValue& InputActionValue);

#pragma endregion

#pragma region Internal Function

	/** 将输入动作绑定到 Owner 的 EnhancedInputComponent；输入资产缺失或 Owner 非本地控制器时放弃绑定 */
	void BindInput();

	/** 依据受控与开启状态注册或移除输入映射上下文 */
	void RefreshInput() const;

	/** 依据受控与开启状态启停驱动仲裁 Tick */
	void RefreshTickEnabled();

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

	/** 将缓存的原始油门/制动/手刹/倒挡按驾驶规则仲裁后写入仿真组件 */
	void ApplyDriveArbitration();

	/** 受控载具的有符号前进速度（km/h），倒退为负 */
	float GetVehicleForwardSpeedKmh() const;

	/** 受控载具的变速箱是否开启自动倒挡（街机倒挡的生效前提） */
	bool IsVehicleAutoReverseEnabled() const;

#pragma endregion
};
