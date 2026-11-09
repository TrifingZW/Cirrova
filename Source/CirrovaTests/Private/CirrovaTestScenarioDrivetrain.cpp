#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
#pragma region T7 驱动布局

	/** T7：RWD / FWD / AWD 静止全油门，验证驱动力只出现在驱动轴。 */
	class FT7DriveLayoutScenario final : public FCirrovaTestScenario
	{
	public:
		FT7DriveLayoutScenario() : FCirrovaTestScenario(TEXT("T7"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			const ECirrovaDriveLayout Layouts[3] = {
				ECirrovaDriveLayout::RWD,
				ECirrovaDriveLayout::FWD,
				ECirrovaDriveLayout::AWD
			};
			const TCHAR* Labels[3] = {TEXT("rwd"), TEXT("fwd"), TEXT("awd")};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.DriveLayout = Layouts[Index];
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(Ground, 0.02f, 0.0f, (Index - 1) * 30.0f),
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
						Vehicle->SetInputs(1.0f, 0.0f, 0.0f, 1);
					}
				}
			}

			if (Elapsed < 5.5f) { return; }

			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.7, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				const float Distance = static_cast<float>((Stat.EndLocation - Stat.StartLocation).Size2D()) * 0.01f;
				AddCheck(Distance > 1.0f, FString::Printf(TEXT("v%d 行驶距离 %.2f m 过小"), Index, Distance));

				const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(Index);
				AddCheck(Latest && Latest->EngineRPM > 800.0f, FString::Printf(TEXT("v%d 发动机转速过低"), Index));

				float WheelForces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 2.7, End);
					WheelForces[WheelIndex] = WheelStat.MeanLongitudinalForce;
				}

				const bool bFrontDriven = Index != 0; // FWD 与 AWD 的前轴驱动
				const bool bRearDriven = Index != 1;  // RWD 与 AWD 的后轴驱动
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const bool bDriven = WheelIndex < 2 ? bFrontDriven : bRearDriven;
					if (bDriven)
					{
						AddCheck(WheelForces[WheelIndex] > 80.0f, FString::Printf(TEXT("v%d 驱动轮 %d 平均驱动力 %.0f N"), Index, WheelIndex, WheelForces[WheelIndex]));
					}
					else
					{
						AddCheck(FMath::Abs(WheelForces[WheelIndex]) < 200.0f, FString::Printf(TEXT("v%d 非驱动轮 %d 平均力 %.0f N"), Index, WheelIndex, WheelForces[WheelIndex]));
					}

					const FString MetricName = FString::Printf(TEXT("wheel%d_force_n"), WheelIndex);
					AddMetric(VehicleMetricName(Index, *MetricName), WheelForces[WheelIndex]);
				}

				const float FrontDelta = FMath::Abs(WheelForces[0] - WheelForces[1]);
				const float RearDelta = FMath::Abs(WheelForces[2] - WheelForces[3]);
				AddMetric(VehicleMetricName(Index, TEXT("front_delta_n")), FrontDelta);
				AddMetric(VehicleMetricName(Index, TEXT("rear_delta_n")), RearDelta);
				AddMetric(VehicleMetricName(Index, TEXT("distance_m")), Distance);
			}

			Complete(TEXT("三种驱动布局均只输出到驱动轴"));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion

#pragma region T8 轮胎松弛

	/** T8：转向阶跃下对比 0 / 0.3 / 1.0 m 侧向松弛。 */
	class FT8RelaxationScenario final : public FCirrovaTestScenario
	{
	public:
		FT8RelaxationScenario() : FCirrovaTestScenario(TEXT("T8"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			const float Relaxation[3] = {0.0f, 0.3f, 1.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.TireRelaxationLength = Relaxation[Index];
				Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(Ground, 0.02f, 0.0f, (Index - 1) * 60.0f),
					Setup
				);
				Harness.SetVehicleLabel(Index, FString::Printf(TEXT("relax_%.1f"), Relaxation[Index]));
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
						Vehicle->SetInputs(0.0f, 0.6f, 0.0f, 0);
						SetHeldVelocity(Vehicle, Vehicle->GetActorForwardVector(), 10.0f);
					}
				}
			}

			if (Elapsed < 5.5f) { return; }

			float Lag[3] = {0.0f, 0.0f, 0.0f};
			float ForceRise[3] = {0.0f, 0.0f, 0.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.5, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				float MaxRaw = 0.0f;
				float MaxRelaxed = 0.0f;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < 2.5) { continue; }
					for (int32 WheelIndex = 0; WheelIndex < 2; ++WheelIndex)
					{
						if (!Sample.Wheels.IsValidIndex(WheelIndex)) { continue; }
						MaxRaw = FMath::Max(MaxRaw, FMath::Abs(Sample.Wheels[WheelIndex].SlipAngleDegrees));
						MaxRelaxed = FMath::Max(MaxRelaxed, FMath::Abs(Sample.Wheels[WheelIndex].RelaxedSlipAngleDegrees));
					}
				}
				AddCheck(MaxRaw > 5.0f, FString::Printf(TEXT("v%d 转向阶跃侧偏角仅 %.1f 度"), Index, MaxRaw));

				// 1) 原始侧偏角首次超过 2 度。
				double StepTime = -1.0;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < 2.5) { continue; }
					float MaxWheel = 0.0f;
					for (int32 WheelIndex = 0; WheelIndex < 2; ++WheelIndex)
					{
						if (Sample.Wheels.IsValidIndex(WheelIndex))
						{
							MaxWheel = FMath::Max(MaxWheel, FMath::Abs(Sample.Wheels[WheelIndex].SlipAngleDegrees));
						}
					}
					if (MaxWheel > 2.0f)
					{
						StepTime = Sample.Time;
						break;
					}
				}

				if (StepTime < 0.0)
				{
					AddFailure(VehicleMetricName(Index, TEXT("no_slip_step")));
					continue;
				}

				// 2) 松弛角与原始角连续 3 帧接近视为建立完成。
				double SettleTime = -1.0;
				int32 StableFrames = 0;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < StepTime) { continue; }
					bool bClose = true;
					for (int32 WheelIndex = 0; WheelIndex < 2; ++WheelIndex)
					{
						if (!Sample.Wheels.IsValidIndex(WheelIndex)) { continue; }
						const float Raw = FMath::Abs(Sample.Wheels[WheelIndex].SlipAngleDegrees);
						const float Relaxed = FMath::Abs(Sample.Wheels[WheelIndex].RelaxedSlipAngleDegrees);
						if (FMath::Abs(Raw - Relaxed) > FMath::Max(Raw * 0.1f, 0.5f)) { bClose = false; break; }
					}
					StableFrames = bClose ? StableFrames + 1 : 0;
					if (StableFrames >= 3)
					{
						SettleTime = Sample.Time;
						break;
					}
				}

				Lag[Index] = SettleTime >= 0.0 ? static_cast<float>(SettleTime - StepTime) : 99.0f;
				AddCheck(SettleTime >= 0.0, VehicleMetricName(Index, TEXT("relaxation_not_settled")));

				// 3) 前轴侧向力达到峰值的 90% 所需时间。
				float PeakForce = 0.0f;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < StepTime) { continue; }
					float Force = 0.0f;
					for (int32 WheelIndex = 0; WheelIndex < 2; ++WheelIndex)
					{
						if (Sample.Wheels.IsValidIndex(WheelIndex))
						{
							Force += FMath::Abs(Sample.Wheels[WheelIndex].TireLateralForce) * 0.5f;
						}
					}
					PeakForce = FMath::Max(PeakForce, Force);
				}

				ForceRise[Index] = 99.0f;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < StepTime) { continue; }
					float Force = 0.0f;
					for (int32 WheelIndex = 0; WheelIndex < 2; ++WheelIndex)
					{
						if (Sample.Wheels.IsValidIndex(WheelIndex))
						{
							Force += FMath::Abs(Sample.Wheels[WheelIndex].TireLateralForce) * 0.5f;
						}
					}
					if (Force >= PeakForce * 0.9f)
					{
						ForceRise[Index] = static_cast<float>(Sample.Time - StepTime);
						break;
					}
				}
				AddCheck(ForceRise[Index] < 90.0f, VehicleMetricName(Index, TEXT("lateral_force_not_built")));

				float MaxLateral = 0.0f;
				float MaxLimit = 0.0f;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < StepTime) { continue; }
					for (const FCirrovaWheelState& Wheel : Sample.Wheels)
					{
						MaxLateral = FMath::Max(MaxLateral, FMath::Abs(Wheel.TireLateralForce));
						MaxLimit = FMath::Max(MaxLimit, Wheel.FrictionForceLimit);
					}
				}
				AddCheck(MaxLateral <= MaxLimit * 1.05f + 20.0f, FString::Printf(TEXT("v%d 侧向力 %.0f N 超出摩擦圆 %.0f N"), Index, MaxLateral, MaxLimit));

				AddMetric(VehicleMetricName(Index, TEXT("lag_s")), Lag[Index]);
				AddMetric(VehicleMetricName(Index, TEXT("force_rise_s")), ForceRise[Index]);
				AddMetric(VehicleMetricName(Index, TEXT("max_raw_alpha_deg")), MaxRaw);
				AddMetric(VehicleMetricName(Index, TEXT("max_relaxed_alpha_deg")), MaxRelaxed);
				AddMetric(VehicleMetricName(Index, TEXT("max_lateral_force_n")), MaxLateral);
			}

			AddCheck(Lag[1] >= Lag[0] - 0.02f, TEXT("0.3 m 松弛的建立时间短于 0 m"));
			AddCheck(Lag[2] >= Lag[1] - 0.02f, TEXT("1.0 m 松弛的建立时间短于 0.3 m"));
			AddCheck(Lag[2] - Lag[0] > 0.03f, TEXT("1.0 m 松弛相对 0 m 没有可测量的滞后"));
			AddCheck(ForceRise[2] >= ForceRise[0] - 0.05f, TEXT("1.0 m 松弛的侧向力建立快于 0 m"));

			Complete(TEXT("松弛长度阶跃响应单调且无发散"));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion

#pragma region T9 空气动力学

	/** T9：保持不同车速对比气动开/关的法向载荷与减速度。 */
	class FT9AerodynamicsScenario final : public FCirrovaTestScenario
	{
	public:
		FT9AerodynamicsScenario() : FCirrovaTestScenario(TEXT("T9"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin(), 400.0f);
			Harness.SpawnGround(Ground);

			for (int32 Index = 0; Index < 2; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.bEnableAerodynamics = Index == 1;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(Ground, 0.02f, 0.0f, Index == 0 ? -30.0f : 30.0f),
					Setup
				);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0); }
				Harness.SetVehicleLabel(Index, Index == 0 ? TEXT("aero_off") : TEXT("aero_on"));
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			const float Speeds[4] = {10.0f, 20.0f, 30.0f, 40.0f};
			if (Elapsed >= 2.5f && Elapsed < 8.5f)
			{
				const int32 Step = FMath::Clamp(FMath::FloorToInt((Elapsed - 2.5f) / 1.5f), 0, 3);
				for (int32 Index = 0; Index < 2; ++Index)
				{
					if (ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(Index))
					{
						Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0);
						SetHeldVelocity(Vehicle, FVector::ForwardVector, Speeds[Step]);
					}
				}
			}

			if (Elapsed < 9.5f) { return; }

			float Loads[2][4] = {};
			float BaseLoads[2] = {};
			float Decelerations[2] = {};
			float TireForceSums[2] = {};
			for (int32 Index = 0; Index < 2; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat AllStat = ComputeVehicleStat(Samples, 0.0, End, FVector::UpVector);
				AddCheck(AllStat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				BaseLoads[Index] = MeanTotalLoad(Samples, 2.0, 2.5);
				for (int32 Step = 0; Step < 4; ++Step)
				{
					const double StepEnd = 2.5 + (Step + 1) * 1.5;
					Loads[Index][Step] = MeanTotalLoad(Samples, StepEnd - 0.5, StepEnd);
				}

				float V85 = 0.0f;
				float V95 = 0.0f;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < 8.5) { V85 = Sample.Velocity.Size() * 0.01f; }
					if (Sample.Time <= 9.4) { V95 = Sample.Velocity.Size() * 0.01f; }
				}
				Decelerations[Index] = V85 - V95;

				float TireForceSum = 0.0f;
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					TireForceSum += ComputeWheelStat(Samples, WheelIndex, 8.0, 8.5).MeanLongitudinalForce;
				}
				TireForceSums[Index] = TireForceSum;

				AddMetric(VehicleMetricName(Index, TEXT("base_load_n")), BaseLoads[Index]);
				AddMetric(VehicleMetricName(Index, TEXT("v85_mps")), V85);
				AddMetric(VehicleMetricName(Index, TEXT("v95_mps")), V95);
				AddMetric(VehicleMetricName(Index, TEXT("decel_mps2")), Decelerations[Index]);
				AddMetric(VehicleMetricName(Index, TEXT("tire_force_40mps_n")), TireForceSum);
				for (int32 Step = 0; Step < 4; ++Step)
				{
					const FString MetricName = FString::Printf(TEXT("load_%dmps_n"), static_cast<int32>(Speeds[Step]));
					AddMetric(VehicleMetricName(Index, *MetricName), Loads[Index][Step]);
				}
			}

			float Delta[4] = {};
			for (int32 Step = 0; Step < 4; ++Step) { Delta[Step] = Loads[1][Step] - Loads[0][Step]; }
			AddCheck(Delta[0] > 20.0f, FString::Printf(TEXT("10 m/s 下压力增量 %.1f N"), Delta[0]));
			AddCheck(Delta[1] > 100.0f, FString::Printf(TEXT("20 m/s 下压力增量 %.1f N"), Delta[1]));
			AddCheck(Delta[3] > 200.0f, FString::Printf(TEXT("40 m/s 下压力增量 %.1f N"), Delta[3]));
			if (Delta[1] > 10.0f)
			{
				const float Ratio = Delta[3] / Delta[1];
				AddCheck(Ratio > 2.5f && Ratio < 6.0f, FString::Printf(TEXT("40/20 m/s 下压力比 %.2f"), Ratio));
				AddMetric(TEXT("downforce_ratio_40_20"), Ratio);
			}
			if (Delta[0] > 10.0f)
			{
				const float Ratio = Delta[2] / Delta[0];
				AddCheck(Ratio > 4.0f && Ratio < 20.0f, FString::Printf(TEXT("30/10 m/s 下压力比 %.2f"), Ratio));
				AddMetric(TEXT("downforce_ratio_30_10"), Ratio);
			}
			// 滑行减速度差直接反映气动阻力；轮胎力在速度被运动学保持时近乎为零，仅作记录。
			AddCheck(Decelerations[1] > Decelerations[0] + 0.2f, FString::Printf(TEXT("气动开/关减速度差 %.2f m/s²"), Decelerations[1] - Decelerations[0]));
			AddMetric(TEXT("drag_tire_force_delta_n"), TireForceSums[1] - TireForceSums[0]);
			AddMetric(TEXT("drag_decel_delta"), Decelerations[1] - Decelerations[0]);

			Complete(TEXT("下压力随速度平方增长，阻力方向正确"));
		}
	};

#pragma endregion

#pragma region T10 多车与性能

	/** T10：4 车与 8 车同场，验证约束槽数量与物理步耗时。 */
	class FT10MultiVehicleScenario final : public FCirrovaTestScenario
	{
	public:
		FT10MultiVehicleScenario() : FCirrovaTestScenario(TEXT("T10"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			GroundTransform = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(GroundTransform);
			SpawnRow(Harness, 0);
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (!bFourVehiclePhaseDone && Elapsed >= 5.0f)
			{
				bFourVehiclePhaseDone = true;
				EvaluatePhase(Harness, 4, 5.0, TEXT("4v"));
				SpawnRow(Harness, 1);
			}

			if (Elapsed >= 10.0f)
			{
				EvaluatePhase(Harness, 8, 10.0, TEXT("8v"));
				Complete(TEXT("4 车 16 约束，8 车 32 约束，物理步无持续增长"));
			}
		}

	private:
		void SpawnRow(FCirrovaTestHarness& Harness, const int32 Row)
		{
			for (int32 Column = 0; Column < 4; ++Column)
			{
				const int32 Index = Row * 4 + Column;
				const float X = (static_cast<float>(Column) - 1.5f) * 8.0f;
				const float Y = (static_cast<float>(Row) - 0.5f) * 6.0f;
				FCirrovaTestVehicleSetup Setup;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(GroundTransform, 0.02f, X, Y),
					Setup
				);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0); }
				Harness.SetVehicleLabel(Index, FString::Printf(TEXT("grid_%d"), Index));
			}
		}

		void EvaluatePhase(
			FCirrovaTestHarness& Harness,
			const int32 VehicleCount,
			const double End,
			const TCHAR* Phase
		)
		{
			for (int32 Index = 0; Index < VehicleCount; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const FVehicleStat Stat = ComputeVehicleStat(Samples, End - 1.0, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, End - 1.0, End);
					if (WheelStat.Count <= 0) { continue; }
					AddCheck(WheelStat.ContactFraction > 0.95f, FString::Printf(TEXT("%s v%d 车轮 %d 接地比例 %.3f"), Phase, Index, WheelIndex, WheelStat.ContactFraction));
					AddCheck(WheelStat.NativeFraction > 0.95f, FString::Printf(TEXT("%s v%d 车轮 %d 原生悬挂比例 %.3f"), Phase, Index, WheelIndex, WheelStat.NativeFraction));
					AddCheck(WheelStat.MeanLoad > 2200.0f && WheelStat.MeanLoad < 2700.0f, FString::Printf(TEXT("%s v%d 车轮 %d 载荷 %.1f N"), Phase, Index, WheelIndex, WheelStat.MeanLoad));
				}
			}

			AddCheck(
				Harness.GetRegisteredVehicleCount() == VehicleCount,
				FString::Printf(TEXT("%s 注册载具数量 %d"), Phase, Harness.GetRegisteredVehicleCount())
			);
			AddCheck(
				Harness.GetActiveWheelConstraintCount() == VehicleCount * 4,
				FString::Printf(TEXT("%s 活动轮胎约束数量 %d"), Phase, Harness.GetActiveWheelConstraintCount())
			);

			const float LastStepMs = Harness.GetLastStepMs();
			const float AverageStepMs = Harness.GetAverageStepMs();
			const float PeakStepMs = Harness.GetPeakStepMs();
			AddCheck(LastStepMs < 12.0f, FString::Printf(TEXT("%s 最近物理步 %.2f ms"), Phase, LastStepMs));
			AddCheck(AverageStepMs < 12.0f, FString::Printf(TEXT("%s 平均物理步 %.2f ms"), Phase, AverageStepMs));
			AddCheck(PeakStepMs < 40.0f, FString::Printf(TEXT("%s 峰值物理步 %.2f ms"), Phase, PeakStepMs));

			const TArray<float>& StepSamples = Harness.GetPhysicsStepMs();
			if (StepSamples.Num() >= 60)
			{
				const int32 LastStart = StepSamples.Num() - 30;
				const int32 PrevStart = FMath::Max(0, LastStart - 30);
				float LastSum = 0.0f;
				float PrevSum = 0.0f;
				for (int32 Sample = LastStart; Sample < StepSamples.Num(); ++Sample) { LastSum += StepSamples[Sample]; }
				for (int32 Sample = PrevStart; Sample < LastStart; ++Sample) { PrevSum += StepSamples[Sample]; }
				const float LastAvg = LastSum / 30.0f;
				const float PrevAvg = PrevSum / FMath::Max(LastStart - PrevStart, 1);
				AddCheck(LastAvg < PrevAvg * 3.0f + 0.5f, FString::Printf(TEXT("%s 物理步后段 %.2f ms，前段 %.2f ms"), Phase, LastAvg, PrevAvg));
				AddMetric(FString::Printf(TEXT("%s_step_last_30_ms"), Phase), LastAvg);
				AddMetric(FString::Printf(TEXT("%s_step_prev_30_ms"), Phase), PrevAvg);
			}

			AddMetric(FString::Printf(TEXT("%s_last_step_ms"), Phase), LastStepMs);
			AddMetric(FString::Printf(TEXT("%s_average_step_ms"), Phase), AverageStepMs);
			AddMetric(FString::Printf(TEXT("%s_peak_step_ms"), Phase), PeakStepMs);
			AddMetric(FString::Printf(TEXT("%s_active_constraints"), Phase), Harness.GetActiveWheelConstraintCount());
			AddMetric(FString::Printf(TEXT("%s_registered_vehicles"), Phase), Harness.GetRegisteredVehicleCount());
		}

		FTransform GroundTransform;
		bool bFourVehiclePhaseDone = false;
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateDrivetrainScenario(const FString& Name)
{
	if (Name == TEXT("T7")) { return MakeUnique<FT7DriveLayoutScenario>(); }
	if (Name == TEXT("T8")) { return MakeUnique<FT8RelaxationScenario>(); }
	if (Name == TEXT("T9")) { return MakeUnique<FT9AerodynamicsScenario>(); }
	if (Name == TEXT("T10")) { return MakeUnique<FT10MultiVehicleScenario>(); }
	return nullptr;
}
