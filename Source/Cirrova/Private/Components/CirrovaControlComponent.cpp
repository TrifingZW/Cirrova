#include "Components/CirrovaControlComponent.h"

#include <EnhancedInputComponent.h>
#include <EnhancedInputSubsystems.h>
#include <InputMappingContext.h>
#include <Components/SingularisMorphVehicleSimulationComponent.h>
#include <Components/SingularisTransmissionComponent.h>
#include <GameFramework/PlayerController.h>
#include <GameFramework/SpringArmComponent.h>
#include <UObject/ConstructorHelpers.h>

UCirrovaControlComponent::UCirrovaControlComponent()
{
	SetIsReplicatedByDefault(false);

	// 仲裁 Tick 默认关闭，受控期间按需启用（见 RefreshTickEnabled）
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;

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

void UCirrovaControlComponent::TickComponent(
	const float DeltaTime,
	const ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction
)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	ApplyDriveArbitration();
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

	// 5) 换车即重置驾驶仲裁状态：倒挡模式与按键缓存不得跨载具泄漏；
	//    受控中的按键会由后续 Triggered 事件重新填充
	RawThrottleInput = 0.0f;
	RawBrakeInput = 0.0f;
	RawHandbrakeInput = 0.0f;
	bRawReverseKeyHeld = false;
	bReverseMode = false;

	// 6) 仲裁 Tick 随受控状态启停
	RefreshTickEnabled();
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

	// 5) 仲裁 Tick 随开启状态启停
	RefreshTickEnabled();
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

	// 转向不经驾驶仲裁：EnhancedInput 的 Triggered 逐帧触发，天然满足逐帧重写语义
	VehicleSimulation->SetInputAxis1D(Chaos::SteeringControlName, InputActionValue.Get<float>());
}

void UCirrovaControlComponent::HandleThrottle(const FInputActionValue& InputActionValue)
{
	// 仅缓存原始值，驱动通道写入由 Tick 仲裁统一完成
	RawThrottleInput = InputActionValue.Get<float>();
}

void UCirrovaControlComponent::HandleThrottleCompleted(const FInputActionValue& InputActionValue)
{
	RawThrottleInput = 0.0f;
}

void UCirrovaControlComponent::HandleBrakeTriggered(const FInputActionValue& InputActionValue)
{
	// 仅缓存原始值，S 在街机倒挡模式下可能被仲裁改写为倒挡油门
	RawBrakeInput = InputActionValue.Get<float>();
}

void UCirrovaControlComponent::HandleBrakeStarted(const FInputActionValue& InputActionValue)
{
	RawBrakeInput = InputActionValue.Get<float>();
}

void UCirrovaControlComponent::HandleBrakeCompleted(const FInputActionValue& InputActionValue)
{
	RawBrakeInput = 0.0f;
}

void UCirrovaControlComponent::HandleHandbrakeStarted(const FInputActionValue& InputActionValue)
{
	RawHandbrakeInput = 1.0f;
}

void UCirrovaControlComponent::HandleHandbrakeCompleted(const FInputActionValue& InputActionValue)
{
	RawHandbrakeInput = 0.0f;
}

void UCirrovaControlComponent::HandleReverseStarted(const FInputActionValue& InputActionValue)
{
	bRawReverseKeyHeld = true;
}

void UCirrovaControlComponent::HandleReverseCompleted(const FInputActionValue& InputActionValue)
{
	bRawReverseKeyHeld = false;
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
	EnhancedInputComponent->BindAction(
		ThrottleInputAction,
		ETriggerEvent::Completed,
		this,
		&UCirrovaControlComponent::HandleThrottleCompleted
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

	// 5) 挡位输入为可选动作：资产缺失时仅跳过对应绑定，不影响基础操控；
	//    倒挡键缓存后经驱动仲裁生效（街机倒挡派生或直通写入），
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

void UCirrovaControlComponent::RefreshTickEnabled()
{
	// 仲裁须逐帧重写驱动通道（输入生产者为整体覆盖语义），
	// 仅受控且开启期间需要 Tick
	SetComponentTickEnabled(bIsEnable && ControlledVehicle.IsValid());
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

void UCirrovaControlComponent::ApplyDriveArbitration()
{
	// 1) 卫语句：未开启或无受控载具时不写任何通道。
	//    输入生产者为整体覆盖语义，停写后各通道于下一物理步自然归零，无需逐项清零
	if (!bIsEnable) return;
	USingularisMorphVehicleSimulationComponent* VehicleSimulation = FindVehicleSimulation();
	if (!IsValid(VehicleSimulation)) return;

	// 2) 街机倒挡状态机。W 按下即退出（W/S 同按时油门优先，避免挡位振荡）；
	//    独立倒挡键按住期间保持；按住 S 且车速低于方向切换速度时切入；
	//    其余情况保持现状（松开 S 后可带挡滑行倒车，与经典载具插件一致）
	const bool bArcadeActive = bReverseAsBrake && IsVehicleAutoReverseEnabled();
	if (bArcadeActive)
	{
		if (RawThrottleInput > KINDA_SMALL_NUMBER)
			bReverseMode = false;
		else if (bRawReverseKeyHeld
			|| (RawBrakeInput > KINDA_SMALL_NUMBER
				&& FMath::Abs(GetVehicleForwardSpeedKmh()) < DirectionSwitchSpeed))
			bReverseMode = true;
	}

	// 3) 路由：未开启街机倒挡时全部直通（保持各键位原始语义）；
	//    街机模式下倒挡控制只跟随模式锁存（倒挡键经状态机进入，W 优先退出），
	//    避免按住倒挡键时 W 被倒挡覆盖
	float Throttle = RawThrottleInput;
	float Brake = RawBrakeInput;
	const bool bReverse = bArcadeActive ? bReverseMode : bRawReverseKeyHeld;

	if (bArcadeActive && bReverseMode)
	{
		// 倒挡中：S 是倒挡油门，不再施加制动（倒挡由持续写入的倒挡控制经 AutoReverse 持有）
		Throttle = RawBrakeInput;
		Brake = 0.0f;
	}
	else if (bArcadeActive && bThrottleAsBrake && RawThrottleInput > KINDA_SMALL_NUMBER
		&& GetVehicleForwardSpeedKmh() < -DirectionSwitchSpeed)
	{
		// 倒车滑行中按 W：转为全刹车直至车速降到方向切换速度以下
		// （W 已在状态机中退出倒挡模式，挡位正经空挡切回前进挡）
		Throttle = 0.0f;
		Brake = 1.0f;
	}

	// 4) 输入生产者每物理步整体覆盖：全部驱动通道须逐帧重写才能保持按住状态
	VehicleSimulation->SetInputAxis1D(Chaos::ThrottleControlName, Throttle);
	VehicleSimulation->SetInputAxis1D(Chaos::BrakeControlName, Brake);
	VehicleSimulation->SetInputAxis1D(Chaos::HandbrakeControlName, RawHandbrakeInput);
	VehicleSimulation->SetInputBool(Chaos::ReverseControlName, bReverse);
}

float UCirrovaControlComponent::GetVehicleForwardSpeedKmh() const
{
	const AActor* Vehicle = ControlledVehicle.Get();
	if (!IsValid(Vehicle)) return 0.0f;

	// 车架前向的有符号速度（cm/s → km/h），负值即倒退
	return FVector::DotProduct(Vehicle->GetVelocity(), Vehicle->GetActorForwardVector()) * 0.036f;
}

bool UCirrovaControlComponent::IsVehicleAutoReverseEnabled() const
{
	const AActor* Vehicle = ControlledVehicle.Get();
	if (!IsValid(Vehicle)) return false;

	const USingularisTransmissionComponent* Transmission =
		Vehicle->FindComponentByClass<USingularisTransmissionComponent>();
	return Transmission && Transmission->AutoReverse;
}
