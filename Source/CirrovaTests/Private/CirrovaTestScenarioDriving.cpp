#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
	/** 单轮窗口内平均转向角（绝对值，度）。 */
	float MeanAbsSteeringDeg(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const int32 WheelIndex,
		const double Start,
		const double End
	)
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (Sample.Time < Start || Sample.Time > End || !Sample.Wheels.IsValidIndex(WheelIndex)) { continue; }
			Sum += FMath::Abs(Sample.Wheels[WheelIndex].SteeringAngleDegrees);
			++Count;
		}
		return Count > 0 ? Sum / static_cast<float>(Count) : 0.0f;
	}

	/** 整车窗口内平均偏航角速度（rad/s，带符号）。 */
	float MeanYawRate(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const double Start,
		const double End
	)
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (Sample.Time < Start || Sample.Time > End) { continue; }
			Sum += FMath::DegreesToRadians(Sample.AngularVelocityDegrees.Z);
			++Count;
		}
		return Count > 0 ? Sum / static_cast<float>(Count) : 0.0f;
	}

#pragma region T19 阿克曼与扭矩矢量

	/** T19：阿克曼内外轮差、扭矩矢量偏航增益与稳态绕圈稳定性。 */
	class FT19SteeringScenario final : public FCirrovaTestScenario
	{
	public:
		FT19SteeringScenario() : FCirrovaTestScenario(TEXT("T19"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin(), 200.0f);
			Harness.SpawnGround(Ground);

			const TCHAR* Labels[3] = {TEXT("ackermann"), TEXT("parallel"), TEXT("vectoring")};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.bEnableAckermannSteering = Index != 1;
				Setup.bEnableTorqueVectoring = Index == 2;
				Setup.TorqueVectoringStrength = 600.0f;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(Ground, 0.02f, 0.0f, (Index - 1) * 40.0f),
					Setup
				);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0); }
				Harness.SetVehicleLabel(Index, Labels[Index]);
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (!bLaunched && Elapsed >= 2.5f)
			{
				bLaunched = true;
				for (int32 Index = 0; Index < 3; ++Index)
				{
					if (ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(Index))
					{
						Vehicle->SetInputs(0.4f, 0.55f, 0.0f, 2);
					}
				}
			}

			if (Elapsed < 9.0f) { return; }

			float YawRates[3] = {0.0f, 0.0f, 0.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 4.0, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));
				AddCheck(Stat.MaxTiltDegrees < 15.0f,
					FString::Printf(TEXT("v%d 最大倾角 %.1f 度"), Index, Stat.MaxTiltDegrees));

				const float SteerLeft = MeanAbsSteeringDeg(Samples, 0, 5.0, End);
				const float SteerRight = MeanAbsSteeringDeg(Samples, 1, 5.0, End);
				const float SteerRear = MeanAbsSteeringDeg(Samples, 2, 5.0, End);
				YawRates[Index] = MeanYawRate(Samples, 5.0, End);

				AddCheck(SteerLeft > 1.0f && SteerRight > 1.0f,
					FString::Printf(TEXT("v%d 前轮转向角不足（%.2f / %.2f 度）"), Index, SteerLeft, SteerRight));
				AddCheck(SteerRear < 0.2f,
					FString::Printf(TEXT("v%d 后轮不可转向（%.2f 度）"), Index, SteerRear));

				if (Index == 0)
				{
					// 左转：内侧左轮转角幅值大于外侧右轮。
					AddCheck(SteerLeft > SteerRight + 0.3f,
						FString::Printf(TEXT("v0 阿克曼内外轮差不足（%.2f / %.2f 度）"), SteerLeft, SteerRight));
				}
				else if (Index == 1)
				{
					AddCheck(FMath::Abs(SteerLeft - SteerRight) < 0.1f,
						FString::Printf(TEXT("v1 平行转向角不一致（%.2f / %.2f 度）"), SteerLeft, SteerRight));
				}

				AddMetric(VehicleMetricName(Index, TEXT("steer_left_deg")), SteerLeft);
				AddMetric(VehicleMetricName(Index, TEXT("steer_right_deg")), SteerRight);
				AddMetric(VehicleMetricName(Index, TEXT("yaw_rate_rad_s")), YawRates[Index]);
			}

			AddCheck(
				FMath::Abs(YawRates[2]) > FMath::Abs(YawRates[0]) * 1.02f,
				FString::Printf(
					TEXT("扭矩矢量未提高偏航率（%.3f vs %.3f rad/s）"),
					YawRates[2],
					YawRates[0]
				)
			);

			Complete(TEXT("阿克曼内外轮差与扭矩矢量偏航增益符合预期"));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion

#pragma region T20 手刹坡道起步

	/** T20：上坡手刹驻车（仅后轴锁止），释放后油门起步。 */
	class FT20HandbrakeHillScenario final : public FCirrovaTestScenario
	{
	public:
		FT20HandbrakeHillScenario() : FCirrovaTestScenario(TEXT("T20"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Slope = Harness.MakeSlopedGround(Harness.GetTestOrigin(), 0.0f, 20.0f);
			Harness.SpawnGround(Slope);
			SlopeGround = Slope;

			FCirrovaTestVehicleSetup Setup;
			Setup.DriveLayout = ECirrovaDriveLayout::AWD;
			ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
				Harness.MakeVehicleOnGround(Slope, 0.02f),
				Setup
			);
			if (Vehicle)
			{
				Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0);
				Vehicle->SetHandbrake(1.0f);
			}
			Harness.SetVehicleLabel(0, TEXT("handbrake_hill"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(0);
			if (!Vehicle) { return; }

			if (!bLaunched && Elapsed >= 3.5f)
			{
				bLaunched = true;
				Vehicle->SetHandbrake(0.0f);
				Vehicle->SetInputs(0.7f, 0.0f, 0.0f, 1);
			}

			if (Elapsed < 8.0f) { return; }

			const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(0);
			if (Samples.IsEmpty())
			{
				AddFailure(TEXT("没有采集到载具遥测"));
				Complete(TEXT(""));
				return;
			}

			// 手刹阶段：后轴锁止、前轴不锁、整车保持。
			const float RearLock = FMath::Max(
				ComputeWheelStat(Samples, 2, 2.0, 3.3).LockedFraction,
				ComputeWheelStat(Samples, 3, 2.0, 3.3).LockedFraction
			);
			const float FrontLock = FMath::Max(
				ComputeWheelStat(Samples, 0, 2.0, 3.3).LockedFraction,
				ComputeWheelStat(Samples, 1, 2.0, 3.3).LockedFraction
			);
			AddCheck(RearLock > 0.8f, FString::Printf(TEXT("手刹后轴锁止帧占比 %.2f"), RearLock));
			AddCheck(FrontLock < 0.2f, FString::Printf(TEXT("手刹前轴误锁止帧占比 %.2f"), FrontLock));

			const FVehicleStat HoldStat = ComputeVehicleStat(
				Samples,
				2.0,
				3.3,
				SlopeGround.GetUnitAxis(EAxis::Z)
			);
			AddCheck(HoldStat.bAllFinite, VehicleMetricName(0, TEXT("non_finite")));
			const float HoldDrift = static_cast<float>(
				(HoldStat.EndLocation - HoldStat.StartLocation).Size2D()) * 0.01f;
			AddCheck(HoldDrift < 0.2f, FString::Printf(TEXT("手刹驻坡漂移 %.3f m"), HoldDrift));
			AddCheck(HoldStat.MeanSpeed < 0.15f,
				FString::Printf(TEXT("手刹驻坡平均速度 %.3f m/s"), HoldStat.MeanSpeed));

			// 起步阶段：沿坡面向上（+X）移动。
			const double End = Samples.Last().Time;
			const FVehicleStat LaunchStat = ComputeVehicleStat(Samples, 4.0, End, FVector::UpVector);
			const FVector Forward = SlopeGround.GetUnitAxis(EAxis::X);
			const FVector Delta = (LaunchStat.EndLocation - LaunchStat.StartLocation) * 0.01f;
			const float Uphill = static_cast<float>(FVector::DotProduct(Delta, Forward));
			AddCheck(Uphill > 1.5f, FString::Printf(TEXT("起步上坡距离 %.2f m"), Uphill));

			AddMetric(TEXT("rear_lock_fraction"), RearLock);
			AddMetric(TEXT("front_lock_fraction"), FrontLock);
			AddMetric(TEXT("hold_drift_m"), HoldDrift);
			AddMetric(TEXT("uphill_distance_m"), Uphill);

			Complete(FString::Printf(TEXT("手刹后轴驻坡，释放后上坡 %.2f m"), Uphill));
		}

	private:
		bool bLaunched = false;
		FTransform SlopeGround = FTransform::Identity;
	};

#pragma endregion

#pragma region T21 双移线

	/** T21：高速双移线，验证转向速率限制下的稳定性与无翻车。 */
	class FT21LaneChangeScenario final : public FCirrovaTestScenario
	{
	public:
		FT21LaneChangeScenario() : FCirrovaTestScenario(TEXT("T21"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin(), 250.0f);
			Harness.SpawnGround(Ground);

			FCirrovaTestVehicleSetup Setup;
			Setup.SteeringRateLimit = 540.0f;
			ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f), Setup);
			if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0); }
			Harness.SetVehicleLabel(0, TEXT("lane_change"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(0);
			if (!Vehicle) { return; }

			if (!bLaunched && Elapsed >= 2.0f)
			{
				bLaunched = true;
				Vehicle->SetInputs(0.3f, 0.0f, 0.0f, 3);
				SetHeldVelocity(Vehicle, Vehicle->GetActorForwardVector(), 20.0f);
			}
			if (bLaunched)
			{
				const float Steering = Elapsed < 2.2f ? 0.0f
					: (Elapsed < 2.55f ? 0.3f
						: (Elapsed < 2.9f ? -0.3f : 0.0f));
				Vehicle->SetInputs(0.3f, Steering, 0.0f, 3);
			}

			if (Elapsed < 8.0f) { return; }

			const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(0);
			if (Samples.IsEmpty())
			{
				AddFailure(TEXT("没有采集到载具遥测"));
				Complete(TEXT(""));
				return;
			}

			const double End = Samples.Last().Time;
			const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.0, End, FVector::UpVector);
			AddCheck(Stat.bAllFinite, VehicleMetricName(0, TEXT("non_finite")));
			AddCheck(Stat.MaxTiltDegrees < 25.0f,
				FString::Printf(TEXT("最大倾角 %.1f 度"), Stat.MaxTiltDegrees));

			const FVector Delta = (Stat.EndLocation - Stat.StartLocation) * 0.01f;
			const float Lateral = FMath::Abs(Delta.Y);
			AddCheck(Lateral > 1.0f, FString::Printf(TEXT("侧向位移 %.2f m 过小"), Lateral));

			const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(0);
			AddCheck(GetSpeedKmh(Latest) / 3.6f > 5.0f,
				FString::Printf(TEXT("最终速度 %.2f m/s"), GetSpeedKmh(Latest) / 3.6f));

			AddMetric(TEXT("lateral_m"), Lateral);
			AddMetric(TEXT("max_tilt_deg"), Stat.MaxTiltDegrees);
			AddMetric(TEXT("final_speed_mps"), GetSpeedKmh(Latest) / 3.6f);

			Complete(FString::Printf(TEXT("双移线侧向位移 %.2f m，无翻车"), Lateral));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion

#pragma region T22 发动机制动曲线

	/** T22：滑行对比发动机制动曲线开/关的减速差异。 */
	class FT22EngineBrakingScenario final : public FCirrovaTestScenario
	{
	public:
		FT22EngineBrakingScenario() : FCirrovaTestScenario(TEXT("T22"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin(), 250.0f);
			Harness.SpawnGround(Ground);

			const TCHAR* Labels[2] = {TEXT("engine_brake_off"), TEXT("engine_brake_on")};
			for (int32 Index = 0; Index < 2; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.bUseEngineBrakingCurve = Index == 1;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(Ground, 0.02f, 0.0f, (Index - 0.5f) * 40.0f),
					Setup
				);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 3); }
				Harness.SetVehicleLabel(Index, Labels[Index]);
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (!bLaunched && Elapsed >= 2.0f)
			{
				bLaunched = true;
				for (int32 Index = 0; Index < 2; ++Index)
				{
					if (ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(Index))
					{
						Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 3);
						SetHeldVelocity(Vehicle, Vehicle->GetActorForwardVector(), 20.0f);
					}
				}
			}

			if (Elapsed < 6.0f) { return; }

			float Speeds[2] = {0.0f, 0.0f};
			for (int32 Index = 0; Index < 2; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.0, Samples.Last().Time, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));
				const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(Index);
				Speeds[Index] = GetSpeedKmh(Latest) / 3.6f;
				AddMetric(VehicleMetricName(Index, TEXT("final_speed_mps")), Speeds[Index]);
			}

			AddCheck(Speeds[0] > Speeds[1] + 0.5f,
				FString::Printf(TEXT("发动机制动曲线未增加减速（%.2f vs %.2f m/s）"), Speeds[0], Speeds[1]));
			AddCheck(Speeds[1] > 5.0f,
				FString::Printf(TEXT("曲线车辆过度减速（%.2f m/s）"), Speeds[1]));

			Complete(FString::Printf(TEXT("发动机制动曲线减速差 %.2f m/s"), Speeds[0] - Speeds[1]));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateDrivingScenario(const FString& Name)
{
	if (Name == TEXT("T19")) { return MakeUnique<FT19SteeringScenario>(); }
	if (Name == TEXT("T20")) { return MakeUnique<FT20HandbrakeHillScenario>(); }
	if (Name == TEXT("T21")) { return MakeUnique<FT21LaneChangeScenario>(); }
	if (Name == TEXT("T22")) { return MakeUnique<FT22EngineBrakingScenario>(); }
	return nullptr;
}
