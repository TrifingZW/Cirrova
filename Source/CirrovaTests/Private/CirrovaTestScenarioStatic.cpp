#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
#pragma region T0 静置

	/** T0：平地静置，验证原生悬挂与轮胎约束的静态平衡。 */
	class FT0StaticScenario final : public FCirrovaTestScenario
	{
	public:
		FT0StaticScenario() : FCirrovaTestScenario(TEXT("T0"), 20.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			FCirrovaTestVehicleSetup Setup;
			ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f), Setup);
			if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0); }
			Harness.SetVehicleLabel(0, TEXT("static"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (Elapsed < 5.0f) { return; }

			const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(0);
			if (Samples.IsEmpty())
			{
				AddFailure(TEXT("没有采集到载具遥测"));
				Complete(TEXT(""));
				return;
			}

			const double End = Samples.Last().Time;
			const double Start = End - 1.0;
			const FVehicleStat VehicleStat = ComputeVehicleStat(Samples, Start, End, FVector::UpVector);
			if (VehicleStat.Count < 30)
			{
				AddFailure(FString::Printf(TEXT("测量窗口样本不足：%d"), VehicleStat.Count));
				Complete(TEXT(""));
				return;
			}

			AddCheck(VehicleStat.bAllFinite, TEXT("遥测出现非有限值"));
			AddCheck(VehicleStat.MeanSpeed < 0.1f, FString::Printf(TEXT("静置平均速度 %.3f m/s 过大"), VehicleStat.MeanSpeed));
			AddCheck(VehicleStat.MaxAbsVerticalSpeed < 0.05f, FString::Printf(TEXT("静置垂直速度 %.3f m/s 过大"), VehicleStat.MaxAbsVerticalSpeed));

			const float DriftXy = static_cast<float>((VehicleStat.EndLocation - VehicleStat.StartLocation).Size2D()) * 0.01f;
			const float DriftZ = FMath::Abs(VehicleStat.EndLocation.Z - VehicleStat.StartLocation.Z) * 0.01f;
			AddCheck(DriftXy < 0.03f, FString::Printf(TEXT("静置水平漂移 %.3f m 过大"), DriftXy));
			AddCheck(DriftZ < 0.01f, FString::Printf(TEXT("静置垂直漂移 %.3f m 过大"), DriftZ));

			const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(0);
			AddCheck(Latest && Latest->bTireCoupling, TEXT("轮胎未以求解器约束耦合"));
			AddCheck(Harness.GetRegisteredVehicleCount() == 1, TEXT("调度器注册载具数量不是 1"));
			AddCheck(Harness.GetActiveWheelConstraintCount() == 4, TEXT("活动轮胎约束数量不是 4"));

			ACirrovaTestVehicle* TestVehicle = Harness.GetVehicle(0);
			AddMetric(TEXT("gravity_z_cm_s2"), Harness.GetWorld() ? Harness.GetWorld()->GetGravityZ() : 0.0f);
			AddMetric(TEXT("chassis_mass_kg"), TestVehicle ? TestVehicle->GetChassisMass() : 0.0f);
			AddMetric(TEXT("body_mass_kg"), TestVehicle ? TestVehicle->GetBodyMass() : 0.0f);

			float LoadSum = 0.0f;
			float SpringSum = 0.0f;
			for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
			{
				const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, Start, End);
				AddCheck(WheelStat.Count > 0, WheelMetricName(WheelIndex, TEXT("no_samples")));
				if (WheelStat.Count <= 0) { continue; }

				AddCheck(WheelStat.ContactFraction > 0.98f, FString::Printf(TEXT("车轮 %d 接地比例 %.3f"), WheelIndex, WheelStat.ContactFraction));
				AddCheck(WheelStat.NativeFraction > 0.98f, FString::Printf(TEXT("车轮 %d 原生悬挂生效比例 %.3f"), WheelIndex, WheelStat.NativeFraction));
				AddCheck(WheelStat.ConstraintFraction > 0.98f, FString::Printf(TEXT("车轮 %d 约束长度使用比例 %.3f"), WheelIndex, WheelStat.ConstraintFraction));
				AddCheck(WheelStat.AppliedFraction > 0.98f, FString::Printf(TEXT("车轮 %d 轮胎约束启用比例 %.3f"), WheelIndex, WheelStat.AppliedFraction));
				AddCheck(WheelStat.ClampFraction < 0.02f, FString::Printf(TEXT("车轮 %d 压缩速度钳制比例 %.3f"), WheelIndex, WheelStat.ClampFraction));
				AddCheck(WheelStat.MeanLoad > 2200.0f && WheelStat.MeanLoad < 2700.0f, FString::Printf(TEXT("车轮 %d 平均法向载荷 %.1f N"), WheelIndex, WheelStat.MeanLoad));
				AddCheck(WheelStat.MeanSpring > 0.27f && WheelStat.MeanSpring < 0.29f, FString::Printf(TEXT("车轮 %d 平均弹簧长度 %.4f m"), WheelIndex, WheelStat.MeanSpring));
				AddCheck(WheelStat.MeanAbsSlip < 0.05f, FString::Printf(TEXT("车轮 %d 平均滑移率 %.4f"), WheelIndex, WheelStat.MeanAbsSlip));
				AddCheck(WheelStat.MeanAbsAlpha < 2.0f, FString::Printf(TEXT("车轮 %d 平均侧偏角 %.2f 度"), WheelIndex, WheelStat.MeanAbsAlpha));
				AddCheck(WheelStat.MeanAbsLateralForce < 100.0f, FString::Printf(TEXT("车轮 %d 平均侧向力 %.1f N"), WheelIndex, WheelStat.MeanAbsLateralForce));

				AddMetric(WheelMetricName(WheelIndex, TEXT("mean_load_n")), WheelStat.MeanLoad);
				AddMetric(WheelMetricName(WheelIndex, TEXT("mean_spring_m")), WheelStat.MeanSpring);
				AddMetric(WheelMetricName(WheelIndex, TEXT("mean_compression_m")), WheelStat.MeanCompression);
				AddMetric(WheelMetricName(WheelIndex, TEXT("mean_slip")), WheelStat.MeanAbsSlip);
				LoadSum += WheelStat.MeanLoad;
				SpringSum += WheelStat.MeanSpring;
			}

			AddMetric(TEXT("mean_load_total_n"), LoadSum);
			AddMetric(TEXT("mean_spring_avg_m"), SpringSum / 4.0f);
			AddMetric(TEXT("drift_xy_m"), DriftXy);
			AddMetric(TEXT("drift_z_m"), DriftZ);
			AddMetric(TEXT("active_constraints"), Harness.GetActiveWheelConstraintCount());
			AddMetric(TEXT("registered_vehicles"), Harness.GetRegisteredVehicleCount());
			AddMetric(TEXT("solver_dt_ms"), Latest ? Latest->SolverDeltaTime * 1000.0f : 0.0f);

			Complete(FString::Printf(
				TEXT("四轮平均载荷 %.1f N，弹簧 %.4f m，水平漂移 %.3f m"),
				LoadSum / 4.0f,
				SpringSum / 4.0f,
				DriftXy
			));
		}
	};

#pragma endregion

#pragma region T1 落地冲击

	/** T1：从 0.5 m 与 2 m 高度落地，验证收敛与峰值有界。 */
	class FT1ImpactScenario final : public FCirrovaTestScenario
	{
	public:
		FT1ImpactScenario() : FCirrovaTestScenario(TEXT("T1"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			FCirrovaTestVehicleSetup Setup;
			ACirrovaTestVehicle* VehicleA = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.5f, -20.0f), Setup);
			if (VehicleA) { VehicleA->SetInputs(0.0f, 0.0f, 1.0f, 0); }
			Harness.SetVehicleLabel(0, TEXT("drop_0_5"));
			ACirrovaTestVehicle* VehicleB = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 2.0f, 20.0f), Setup);
			if (VehicleB) { VehicleB->SetInputs(0.0f, 0.0f, 1.0f, 0); }
			Harness.SetVehicleLabel(1, TEXT("drop_2_0"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (Elapsed < 7.0f) { return; }

			for (int32 VehicleIndex = 0; VehicleIndex < 2; ++VehicleIndex)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(VehicleIndex);
				if (Samples.IsEmpty())
				{
					AddFailure(VehicleMetricName(VehicleIndex, TEXT("no_samples")));
					continue;
				}

				const double End = Samples.Last().Time;
				const float ImpactTime = FirstContactTime(Samples);
				AddCheck(ImpactTime >= 0.0f, VehicleMetricName(VehicleIndex, TEXT("no_contact")));

				if (ImpactTime >= 0.0f)
				{
					const float SettledTime = FindSettledTime(Samples, ImpactTime, 0.15f, 1.0f);
					AddCheck(SettledTime >= 0.0f, VehicleMetricName(VehicleIndex, TEXT("not_settled")));
					if (SettledTime >= 0.0f)
					{
						AddCheck(
							SettledTime - ImpactTime < 3.5f,
							FString::Printf(TEXT("v%d 收敛时间 %.2f s"), VehicleIndex, SettledTime - ImpactTime)
						);
						AddMetric(VehicleMetricName(VehicleIndex, TEXT("settle_s")), SettledTime - ImpactTime);
					}
				}

				float MaxCompression = 0.0f;
				float MaxLoad = 0.0f;
				int32 ClampSamples = 0;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					for (const FCirrovaWheelState& Wheel : Sample.Wheels)
					{
						MaxCompression = FMath::Max(MaxCompression, Wheel.Compression);
						MaxLoad = FMath::Max(MaxLoad, Wheel.NormalLoad);
						ClampSamples += Wheel.bCompressionVelocityClamped ? 1 : 0;
					}
				}

				const FVehicleStat AllStat = ComputeVehicleStat(Samples, 0.0, End, FVector::UpVector);
				AddCheck(AllStat.bAllFinite, VehicleMetricName(VehicleIndex, TEXT("non_finite")));
				AddCheck(MaxCompression < 0.18f, FString::Printf(TEXT("v%d 峰值压缩 %.3f m"), VehicleIndex, MaxCompression));
				AddCheck(MaxLoad < 40000.0f, FString::Printf(TEXT("v%d 峰值载荷 %.0f N"), VehicleIndex, MaxLoad));
				AddCheck(AllStat.MaxTiltDegrees < 30.0f, FString::Printf(TEXT("v%d 最大倾角 %.1f 度"), VehicleIndex, AllStat.MaxTiltDegrees));
				AddCheck(AllStat.MaxAbsVerticalSpeed < 10.0f, FString::Printf(TEXT("v%d 最大垂直速度 %.1f m/s"), VehicleIndex, AllStat.MaxAbsVerticalSpeed));

				const double FinalStart = End - 1.0;
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, FinalStart, End);
					if (WheelStat.Count <= 0) { continue; }

					AddCheck(WheelStat.ContactFraction > 0.95f, FString::Printf(TEXT("v%d 车轮 %d 最终接地比例 %.3f"), VehicleIndex, WheelIndex, WheelStat.ContactFraction));
					AddCheck(WheelStat.NativeFraction > 0.95f, FString::Printf(TEXT("v%d 车轮 %d 最终原生悬挂比例 %.3f"), VehicleIndex, WheelIndex, WheelStat.NativeFraction));
					AddCheck(WheelStat.MeanLoad > 2200.0f && WheelStat.MeanLoad < 2700.0f, FString::Printf(TEXT("v%d 车轮 %d 最终载荷 %.1f N"), VehicleIndex, WheelIndex, WheelStat.MeanLoad));
					AddCheck(WheelStat.MeanSpring > 0.27f && WheelStat.MeanSpring < 0.29f, FString::Printf(TEXT("v%d 车轮 %d 最终弹簧 %.4f m"), VehicleIndex, WheelIndex, WheelStat.MeanSpring));
					AddCheck(WheelStat.MeanAbsCompressionVelocity < 0.15f, FString::Printf(TEXT("v%d 车轮 %d 最终压缩速度 %.3f m/s"), VehicleIndex, WheelIndex, WheelStat.MeanAbsCompressionVelocity));
					AddCheck(WheelStat.ClampFraction < 0.05f, FString::Printf(TEXT("v%d 车轮 %d 最终钳制比例 %.3f"), VehicleIndex, WheelIndex, WheelStat.ClampFraction));
				}

				AddMetric(VehicleMetricName(VehicleIndex, TEXT("peak_compression_m")), MaxCompression);
				AddMetric(VehicleMetricName(VehicleIndex, TEXT("peak_load_n")), MaxLoad);
				AddMetric(VehicleMetricName(VehicleIndex, TEXT("clamp_samples")), ClampSamples);
				AddMetric(VehicleMetricName(VehicleIndex, TEXT("max_tilt_deg")), AllStat.MaxTiltDegrees);
				AddMetric(VehicleMetricName(VehicleIndex, TEXT("max_vz_mps")), AllStat.MaxAbsVerticalSpeed);
			}

			Complete(TEXT("落地冲击在容差内收敛"));
		}
	};

#pragma endregion

#pragma region T2 制动锁止

	/** T2：60 km/h 直线全制动，验证摩擦圆与停止行为。 */
	class FT2BrakingScenario final : public FCirrovaTestScenario
	{
	public:
		FT2BrakingScenario() : FCirrovaTestScenario(TEXT("T2"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			FCirrovaTestVehicleSetup Setup;
			Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f), Setup);
			Harness.SetVehicleLabel(0, TEXT("brake_60"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(0);
			if (!Vehicle) { return; }

			if (!bLaunched && Elapsed >= 3.0f)
			{
				bLaunched = true;
				Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0);
				SetHeldVelocity(Vehicle, Vehicle->GetActorForwardVector(), 60.0f / 3.6f);
			}

			if (!bReleased && Elapsed >= 6.5f)
			{
				bReleased = true;
				Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0);
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
			const FVehicleStat Braking = ComputeVehicleStat(Samples, 3.1, End, FVector::UpVector);
			AddCheck(Braking.bAllFinite, TEXT("遥测出现非有限值"));
			AddCheck(Braking.Count > 0, TEXT("制动窗口没有样本"));

			const float StopDistance = static_cast<float>((Braking.EndLocation - Braking.StartLocation).Size2D()) * 0.01f;
			AddCheck(StopDistance < 40.0f, FString::Printf(TEXT("制动距离 %.1f m 过大"), StopDistance));

			const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(0);
			const float FinalSpeed = GetSpeedKmh(Latest) / 3.6f;
			AddCheck(FinalSpeed < 0.5f, FString::Printf(TEXT("最终速度 %.2f m/s"), FinalSpeed));

			const float FinalOmega = MeanWheelOmega(Samples, End - 0.5, End);
			AddCheck(FinalOmega < 0.5f, FString::Printf(TEXT("最终车轮角速度 %.3f rad/s"), FinalOmega));

			for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
			{
				const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 3.1, End);
				if (WheelStat.Count <= 0) { continue; }

				AddCheck(WheelStat.MaxUtilization <= 1.02f, FString::Printf(TEXT("车轮 %d 最大摩擦利用 %.3f"), WheelIndex, WheelStat.MaxUtilization));
				AddCheck(WheelStat.MaxAbsLongitudinalForce <= WheelStat.MaxFrictionLimit * 1.05f + 10.0f, FString::Printf(TEXT("车轮 %d 纵向力 %.0f N 超出摩擦圆 %.0f N"), WheelIndex, WheelStat.MaxAbsLongitudinalForce, WheelStat.MaxFrictionLimit));

				const FWheelStat EarlyBraking = ComputeWheelStat(Samples, WheelIndex, 3.1, 5.0);
				AddCheck(EarlyBraking.MeanLongitudinalForce < -200.0f, FString::Printf(TEXT("车轮 %d 平均制动力 %.0f N"), WheelIndex, EarlyBraking.MeanLongitudinalForce));
				AddCheck(EarlyBraking.MeanConstraintWheelInverseMass < 0.005f, FString::Printf(TEXT("车轮 %d 制动期间轮端未锁止（R²/J=%.4f）"), WheelIndex, EarlyBraking.MeanConstraintWheelInverseMass));

				const int32 SignChanges = CountSignChanges(Samples, WheelIndex, End - 1.0, End, 50.0f);
				AddCheck(SignChanges <= 4, FString::Printf(TEXT("车轮 %d 最终纵向力符号震荡 %d 次"), WheelIndex, SignChanges));
			}

			AddMetric(TEXT("stop_distance_m"), StopDistance);
			AddMetric(TEXT("final_speed_mps"), FinalSpeed);
			AddMetric(TEXT("final_omega_rad_s"), FinalOmega);
			Complete(FString::Printf(TEXT("制动距离 %.1f m，最终速度 %.2f m/s"), StopDistance, FinalSpeed));
		}

	private:
		bool bLaunched = false;
		bool bReleased = false;
	};

#pragma endregion

#pragma region T3 直线驱动

	/** T3：静止全油门 1 挡，验证驱动力对称与滑移有界。 */
	class FT3AccelerationScenario final : public FCirrovaTestScenario
	{
	public:
		FT3AccelerationScenario() : FCirrovaTestScenario(TEXT("T3"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			FCirrovaTestVehicleSetup Setup;
			ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f), Setup);
			if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0); }
			Harness.SetVehicleLabel(0, TEXT("awd_gear1"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(0);
			if (!Vehicle) { return; }

			if (!bLaunched && Elapsed >= 3.0f)
			{
				bLaunched = true;
				Vehicle->SetInputs(1.0f, 0.0f, 0.0f, 1);
			}

			if (Elapsed < 6.0f) { return; }

			const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(0);
			if (Samples.IsEmpty())
			{
				AddFailure(TEXT("没有采集到载具遥测"));
				Complete(TEXT(""));
				return;
			}

			const double End = Samples.Last().Time;
			const FVehicleStat Stat = ComputeVehicleStat(Samples, 3.1, End, FVector::UpVector);
			AddCheck(Stat.bAllFinite, TEXT("遥测出现非有限值"));

			const float Distance = static_cast<float>((Stat.EndLocation - Stat.StartLocation).Size2D()) * 0.01f;
			AddCheck(Distance > 3.0f, FString::Printf(TEXT("行驶距离 %.2f m 过小"), Distance));

			const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(0);
			AddCheck(Latest && Latest->EngineRPM > 800.0f, TEXT("发动机转速过低"));
			AddCheck(Latest && Latest->EngineRPM < 6500.0f, TEXT("发动机转速过高"));

			float FrontForceSum = 0.0f;
			float RearForceSum = 0.0f;
			for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
			{
				const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 3.1, End);
				if (WheelStat.Count <= 0) { continue; }

				AddCheck(WheelStat.MeanLongitudinalForce > 80.0f, FString::Printf(TEXT("车轮 %d 平均驱动力 %.0f N"), WheelIndex, WheelStat.MeanLongitudinalForce));
				AddCheck(WheelStat.MeanConstraintWheelInverseMass > 0.0f, FString::Printf(TEXT("车轮 %d 轮端自由度未进入约束（R²/J=%.4f）"), WheelIndex, WheelStat.MeanConstraintWheelInverseMass));
				AddCheck(WheelStat.MeanAbsSlip < 30.0f, FString::Printf(TEXT("车轮 %d 平均滑移率 %.2f"), WheelIndex, WheelStat.MeanAbsSlip));
				AddCheck(WheelStat.ContactFraction > 0.5f, FString::Printf(TEXT("车轮 %d 接地比例 %.3f"), WheelIndex, WheelStat.ContactFraction));

				if (WheelIndex < 2) { FrontForceSum += WheelStat.MeanLongitudinalForce; }
				else { RearForceSum += WheelStat.MeanLongitudinalForce; }
			}

			const FWheelStat FrontLeft = ComputeWheelStat(Samples, 0, 3.1, End);
			const FWheelStat FrontRight = ComputeWheelStat(Samples, 1, 3.1, End);
			const FWheelStat RearLeft = ComputeWheelStat(Samples, 2, 3.1, End);
			const FWheelStat RearRight = ComputeWheelStat(Samples, 3, 3.1, End);
			const float FrontDelta = FMath::Abs(FrontLeft.MeanLongitudinalForce - FrontRight.MeanLongitudinalForce);
			const float RearDelta = FMath::Abs(RearLeft.MeanLongitudinalForce - RearRight.MeanLongitudinalForce);
			const float FrontScale = FMath::Max(FMath::Abs(FrontLeft.MeanLongitudinalForce), FMath::Abs(FrontRight.MeanLongitudinalForce));
			const float RearScale = FMath::Max(FMath::Abs(RearLeft.MeanLongitudinalForce), FMath::Abs(RearRight.MeanLongitudinalForce));
			AddCheck(FrontDelta <= FrontScale * 0.35f + 100.0f, FString::Printf(TEXT("前轴左右力差 %.0f N"), FrontDelta));
			AddCheck(RearDelta <= RearScale * 0.35f + 100.0f, FString::Printf(TEXT("后轴左右力差 %.0f N"), RearDelta));

			float MaxSurfaceSpeed = 0.0f;
			for (const FCirrovaTestVehicleSample& Sample : Samples)
			{
				for (const FCirrovaWheelState& Wheel : Sample.Wheels)
				{
					MaxSurfaceSpeed = FMath::Max(MaxSurfaceSpeed, FMath::Abs(Wheel.SurfaceSpeed));
				}
			}
			AddCheck(MaxSurfaceSpeed < 60.0f, FString::Printf(TEXT("轮面速度 %.1f m/s 失控"), MaxSurfaceSpeed));

			AddMetric(TEXT("distance_m"), Distance);
			AddMetric(TEXT("engine_rpm"), Latest ? Latest->EngineRPM : 0.0f);
			AddMetric(TEXT("front_force_n"), FrontForceSum);
			AddMetric(TEXT("rear_force_n"), RearForceSum);
			AddMetric(TEXT("front_delta_n"), FrontDelta);
			AddMetric(TEXT("rear_delta_n"), RearDelta);
			AddMetric(TEXT("max_surface_speed_mps"), MaxSurfaceSpeed);
			Complete(FString::Printf(TEXT("行驶 %.2f m，发动机 %.0f RPM"), Distance, Latest ? Latest->EngineRPM : 0.0f));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateStaticScenario(const FString& Name)
{
	if (Name == TEXT("T0")) { return MakeUnique<FT0StaticScenario>(); }
	if (Name == TEXT("T1")) { return MakeUnique<FT1ImpactScenario>(); }
	if (Name == TEXT("T2")) { return MakeUnique<FT2BrakingScenario>(); }
	if (Name == TEXT("T3")) { return MakeUnique<FT3AccelerationScenario>(); }
	return nullptr;
}
