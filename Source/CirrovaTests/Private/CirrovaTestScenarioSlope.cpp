#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
#pragma region T4 侧向斜坡

	/** T4：15/25/35 度侧向斜坡无输入，验证静态侧向保持。 */
	class FT4LateralSlopeScenario final : public FCirrovaTestScenario
	{
	public:
		FT4LateralSlopeScenario() : FCirrovaTestScenario(TEXT("T4"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const float Angles[3] = {15.0f, 25.0f, 35.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const FVector Center = Harness.GetTestOrigin() + FVector(0.0f, Index * 50000.0f, 0.0f);
				const FTransform Ground = Harness.MakeSlopedGround(Center, Angles[Index], 0.0f);
				GroundTransforms.Add(Ground);
				Harness.SpawnGround(Ground);

				FCirrovaTestVehicleSetup Setup;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f), Setup);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0); }
				Harness.SetVehicleLabel(Index, FString::Printf(TEXT("slope_%.0f"), Angles[Index]));
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (Elapsed < 7.0f) { return; }

			const float Angles[3] = {15.0f, 25.0f, 35.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty() || !GroundTransforms.IsValidIndex(Index)) { continue; }

				const FTransform& Ground = GroundTransforms[Index];
				const FVector Normal = Ground.GetUnitAxis(EAxis::Z);
				const FVector Right = Ground.GetUnitAxis(EAxis::Y);
				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 5.0, End, Normal);

				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));
				AddCheck(Stat.Count > 0, VehicleMetricName(Index, TEXT("no_samples")));
				AddCheck(Stat.MinUpDot > 0.98f, FString::Printf(TEXT("v%d 车身未贴合坡面，最小上向量点积 %.3f"), Index, Stat.MinUpDot));

				const FVector Delta = (Stat.EndLocation - Stat.StartLocation) * 0.01f;
				const float DriftLateral = FMath::Abs(FVector::DotProduct(Delta, Right));
				const float DriftTotal = static_cast<float>(Delta.Size2D());
				const float EndSpeed = Stat.MeanSpeed;

				const float Limit = 0.3f;
				const float LimitTotal = 0.3f;
				AddCheck(DriftLateral < Limit, FString::Printf(TEXT("v%d 侧向蠕滑 %.3f m"), Index, DriftLateral));
				AddCheck(DriftTotal < LimitTotal, FString::Printf(TEXT("v%d 面内总蠕滑 %.3f m"), Index, DriftTotal));
				AddCheck(EndSpeed < 0.25f, FString::Printf(TEXT("v%d 测量窗平均速度 %.3f m/s"), Index, EndSpeed));

				const float Theta = FMath::DegreesToRadians(Angles[Index]);
				const float ExpectedLoad = 1000.0f * Gravity * FMath::Cos(Theta);
				AddCheck(FMath::Abs(Stat.MeanSumLoad - ExpectedLoad) < ExpectedLoad * 0.1f, FString::Printf(TEXT("v%d 总法向载荷 %.0f N（期望 %.0f）"), Index, Stat.MeanSumLoad, ExpectedLoad));

				float LateralSum = 0.0f;
				float UtilizationSum = 0.0f;
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 5.0, End);
					if (WheelStat.Count <= 0) { continue; }
					AddCheck(WheelStat.ContactFraction > 0.95f, FString::Printf(TEXT("v%d 车轮 %d 接地比例 %.3f"), Index, WheelIndex, WheelStat.ContactFraction));
					AddCheck(WheelStat.NativeFraction > 0.95f, FString::Printf(TEXT("v%d 车轮 %d 原生悬挂比例 %.3f"), Index, WheelIndex, WheelStat.NativeFraction));
					LateralSum += WheelStat.MeanAbsLateralForce;
					UtilizationSum += WheelStat.MeanUtilization;
				}

				// 原生悬挂约束沿地面法线作用，在斜坡上已提供部分侧向支撑；
				// P0 以零蠕滑为通过标准，轮胎侧向力仅作诊断记录。
				const float ExpectedLateralSum = 1000.0f * Gravity * FMath::Sin(Theta);
				const float ExpectedUtilization = FMath::Tan(Theta);
				const float MeanUtilization = UtilizationSum / 4.0f;
				AddMetric(VehicleMetricName(Index, TEXT("expected_lateral_force_n")), ExpectedLateralSum);
				AddMetric(VehicleMetricName(Index, TEXT("expected_utilization")), ExpectedUtilization);
				AddCheck(MeanUtilization < 0.99f, FString::Printf(TEXT("v%d 平均摩擦利用 %.3f，轮胎已饱和滑移"), Index, MeanUtilization));

				AddMetric(VehicleMetricName(Index, TEXT("drift_lateral_m")), DriftLateral);
				AddMetric(VehicleMetricName(Index, TEXT("drift_total_m")), DriftTotal);
				AddMetric(VehicleMetricName(Index, TEXT("mean_speed_mps")), EndSpeed);
				AddMetric(VehicleMetricName(Index, TEXT("lateral_force_n")), LateralSum);
				AddMetric(VehicleMetricName(Index, TEXT("utilization")), MeanUtilization);
				AddMetric(VehicleMetricName(Index, TEXT("sum_load_n")), Stat.MeanSumLoad);
			}

			Complete(TEXT("侧向斜坡静态保持"));
		}

	private:
		TArray<FTransform> GroundTransforms;
	};

#pragma endregion

#pragma region T5 制动驻坡

	/** T5：15/25/35 度下坡全制动，验证静态驻车与车轮不反转。 */
	class FT5SlopeBrakeScenario final : public FCirrovaTestScenario
	{
	public:
		FT5SlopeBrakeScenario() : FCirrovaTestScenario(TEXT("T5"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const float Angles[3] = {15.0f, 25.0f, 35.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const FVector Center = Harness.GetTestOrigin() + FVector(0.0f, Index * 50000.0f, 0.0f);
				const FTransform Ground = Harness.MakeSlopedGround(Center, 0.0f, -Angles[Index]);
				GroundTransforms.Add(Ground);
				Harness.SpawnGround(Ground);

				FCirrovaTestVehicleSetup Setup;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f), Setup);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0); }
				Harness.SetVehicleLabel(Index, FString::Printf(TEXT("brake_slope_%.0f"), Angles[Index]));
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			for (int32 Index = 0; Index < 3; ++Index)
			{
				if (ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(Index))
				{
					Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0);
				}
			}

			if (Elapsed < 7.0f) { return; }

			const float Angles[3] = {15.0f, 25.0f, 35.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty() || !GroundTransforms.IsValidIndex(Index)) { continue; }

				const FTransform& Ground = GroundTransforms[Index];
				const FVector Normal = Ground.GetUnitAxis(EAxis::Z);
				const FVector Forward = Ground.GetUnitAxis(EAxis::X);
				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 5.0, End, Normal);

				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));
				AddCheck(Stat.Count > 0, VehicleMetricName(Index, TEXT("no_samples")));

				const FVector Delta = (Stat.EndLocation - Stat.StartLocation) * 0.01f;
				const float DriftDownhill = FMath::Abs(FVector::DotProduct(Delta, Forward));
				const float Limit = 0.2f;
				AddCheck(DriftDownhill < Limit, FString::Printf(TEXT("v%d 下坡滑移 %.3f m"), Index, DriftDownhill));
				AddCheck(Stat.MeanSpeed < 0.15f, FString::Printf(TEXT("v%d 平均速度 %.3f m/s"), Index, Stat.MeanSpeed));

				const float Omega = MeanWheelOmega(Samples, 5.0, End);
				AddCheck(Omega < 0.5f, FString::Printf(TEXT("v%d 车轮平均角速度 %.3f rad/s"), Index, Omega));

				const float Theta = FMath::DegreesToRadians(Angles[Index]);
				const float ExpectedLoad = 1000.0f * Gravity * FMath::Cos(Theta);
				const float ExpectedForceSum = -1000.0f * Gravity * FMath::Sin(Theta);
				AddCheck(FMath::Abs(Stat.MeanSumLoad - ExpectedLoad) < ExpectedLoad * 0.1f, FString::Printf(TEXT("v%d 总法向载荷 %.0f N（期望 %.0f）"), Index, Stat.MeanSumLoad, ExpectedLoad));

				float ForceSum = 0.0f;
				float MaxUtilization = 0.0f;
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 5.0, End);
					if (WheelStat.Count <= 0) { continue; }
					AddCheck(WheelStat.MeanAbsOmega < 0.5f, FString::Printf(TEXT("v%d 车轮 %d 驻车角速度 %.3f rad/s"), Index, WheelIndex, WheelStat.MeanAbsOmega));
					AddCheck(WheelStat.LockedFraction > 0.9f, FString::Printf(TEXT("v%d 车轮 %d 制动期间锁止帧占比 %.2f 过低"), Index, WheelIndex, WheelStat.LockedFraction));
					ForceSum += WheelStat.MeanLongitudinalForce;
					MaxUtilization = FMath::Max(MaxUtilization, WheelStat.MaxUtilization);
				}
				// 等扭矩制动在载荷转移下会使前轮先达到局部摩擦上限；P0 以整车无持续加速滑移为准，
				// 局部饱和作为诊断记录，制动分配优化留给 P1。
				AddMetric(VehicleMetricName(Index, TEXT("max_utilization")), MaxUtilization);
				// 制动力由 1D 域与轮胎约束共同承担；P0 以无持续加速滑移、车轮不反转与零速为通过标准。
				AddMetric(VehicleMetricName(Index, TEXT("expected_long_force_n")), ExpectedForceSum);

				AddMetric(VehicleMetricName(Index, TEXT("drift_downhill_m")), DriftDownhill);
				AddMetric(VehicleMetricName(Index, TEXT("mean_speed_mps")), Stat.MeanSpeed);
				AddMetric(VehicleMetricName(Index, TEXT("mean_omega_rad_s")), Omega);
				AddMetric(VehicleMetricName(Index, TEXT("sum_load_n")), Stat.MeanSumLoad);
				AddMetric(VehicleMetricName(Index, TEXT("long_force_n")), ForceSum);
			}

			Complete(TEXT("下坡制动驻车"));
		}

	private:
		TArray<FTransform> GroundTransforms;
	};

#pragma endregion

#pragma region T6 台阶

	/** T6：10/15 cm 台阶，10/40 km/h，圆柱包络/球体扫掠与射线对照。 */
	class FT6StepScenario final : public FCirrovaTestScenario
	{
	public:
		FT6StepScenario() : FCirrovaTestScenario(TEXT("T6"), 60.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			TestOrigin = Harness.GetTestOrigin();
			const FTransform Ground = Harness.MakeFlatGround(TestOrigin);
			Harness.SpawnGround(Ground);

			const float Heights[2] = {0.10f, 0.15f};
			const float Speeds[2] = {10.0f, 40.0f};
			const TCHAR* ProbeNames[3] = {TEXT("cylinder"), TEXT("sphere"), TEXT("ray")};
			const ECirrovaWheelProbeMode ProbeModes[3] = {
				ECirrovaWheelProbeMode::Cylinder,
				ECirrovaWheelProbeMode::Sphere,
				ECirrovaWheelProbeMode::Ray
			};
			constexpr int32 ProbeCount = 3;
			constexpr int32 TotalVehicles = 2 * 2 * ProbeCount;
			constexpr float LaneSpacingM = 15.0f;
			for (int32 HeightIndex = 0; HeightIndex < 2; ++HeightIndex)
			{
				for (int32 SpeedIndex = 0; SpeedIndex < 2; ++SpeedIndex)
				{
					for (int32 ProbeIndex = 0; ProbeIndex < ProbeCount; ++ProbeIndex)
					{
						const int32 Index = HeightIndex * (2 * ProbeCount) + SpeedIndex * ProbeCount + ProbeIndex;
						const float LaneY =
							(static_cast<float>(Index) - (TotalVehicles - 1) * 0.5f) * LaneSpacingM;
						const float HeightCm = Heights[HeightIndex] * 100.0f;
						const bool bEnforced = ProbeIndex != 2;

						// 台阶必须与测试地面在同一世界位置；测试原点远离默认关卡出生点。
						Harness.SpawnGround(Harness.MakeBox(
							TestOrigin + FVector(0.0f, LaneY * 100.0f, HeightCm * 0.5f),
							FVector(400.0f, 800.0f, HeightCm)
						));

						FCirrovaTestVehicleSetup Setup;
						Setup.ChassisExtentCm = FVector(160.0f, 90.0f, 35.0f);
						Setup.ProbeMode = ProbeModes[ProbeIndex];
						Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Ground, 0.02f, -8.0f, LaneY), Setup);
						Harness.SetVehicleLabel(
							Index,
							FString::Printf(TEXT("h%02.0f_v%02.0f_%s"), Heights[HeightIndex] * 100.0f, Speeds[SpeedIndex], ProbeNames[ProbeIndex])
						);

						LaneYs.Add(LaneY);
						StepHeights.Add(Heights[HeightIndex]);
						SpeedsMps.Add(Speeds[SpeedIndex] / 3.6f);
						bEnforcedModes.Add(bEnforced);
					}
				}
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (!bLaunched && Elapsed >= 2.5f)
			{
				bLaunched = true;
				for (int32 Index = 0; Index < Harness.GetVehicleCount() && Index < SpeedsMps.Num(); ++Index)
				{
					if (ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(Index))
					{
						Vehicle->SetInputs(0.0f, 0.0f, 0.0f, 0);
						SetHeldVelocity(Vehicle, FVector::ForwardVector, SpeedsMps[Index]);
					}
				}
			}

			if (Elapsed < 9.5f) { return; }

			for (int32 Index = 0; Index < Harness.GetVehicleCount(); ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty() || !LaneYs.IsValidIndex(Index) || !StepHeights.IsValidIndex(Index)
					|| !bEnforcedModes.IsValidIndex(Index))
				{
					continue;
				}

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 0.0, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				const float FinalX = Stat.EndLocation.X * 0.01f;
				const float StepTop = StepHeights[Index];
				const float StepCenterX = TestOrigin.X * 0.01f;
				const float StepCenterY = TestOrigin.Y * 0.01f + LaneYs[Index];
				const float StepMinX = StepCenterX - 2.0f;
				const float StepMaxX = StepCenterX + 2.0f;
				const float StepMinY = StepCenterY - 4.0f;
				const float StepMaxY = StepCenterY + 4.0f;
				const FVector LocalExtent(160.0f, 90.0f, 35.0f);

				float MaxPenetration = 0.0f;
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					FVector Min;
					FVector Max;
					ComputeChassisAabb(Sample, LocalExtent, Min, Max);

					const bool bOverlapX = Max.X >= StepMinX * 100.0f && Min.X <= StepMaxX * 100.0f;
					const bool bOverlapY = Max.Y >= StepMinY * 100.0f && Min.Y <= StepMaxY * 100.0f;
					if (!bOverlapX || !bOverlapY) { continue; }

					const float Penetration = (TestOrigin.Z + StepTop * 100.0f - Min.Z) * 0.01f;
					MaxPenetration = FMath::Max(MaxPenetration, Penetration);
				}

				AddMetric(VehicleMetricName(Index, TEXT("final_x_m")), FinalX);
				AddMetric(VehicleMetricName(Index, TEXT("final_x_relative_m")), FinalX - StepCenterX);
				AddMetric(VehicleMetricName(Index, TEXT("max_penetration_m")), MaxPenetration);
				AddMetric(VehicleMetricName(Index, TEXT("max_vz_mps")), Stat.MaxAbsVerticalSpeed);
				AddMetric(VehicleMetricName(Index, TEXT("min_up_dot")), Stat.MinUpDot);

				if (!bEnforcedModes[Index])
				{
					// 射线组为对照组：只记录，不作为 P0 通过条件。
					continue;
				}

				AddCheck(Stat.MinUpDot > 0.5f, FString::Printf(TEXT("v%d 翻车/大角度倾斜，最小上向量点积 %.3f"), Index, Stat.MinUpDot));
				AddCheck(Stat.MaxAbsVerticalSpeed < 6.0f, FString::Printf(TEXT("v%d 垂直速度 %.2f m/s"), Index, Stat.MaxAbsVerticalSpeed));
				AddCheck(FinalX > StepMaxX - 0.1f, FString::Printf(TEXT("v%d 未到达台阶后沿，最终 x %.2f m（台阶后沿 %.2f m）"), Index, FinalX, StepMaxX));
				AddCheck(MaxPenetration <= 0.05f, FString::Printf(TEXT("v%d 底盘穿透台阶 %.3f m"), Index, MaxPenetration));
			}

			Complete(TEXT("台阶通过（圆柱包络/球体扫掠判定，射线为对照）"));
		}

	private:
		bool bLaunched = false;
		FVector TestOrigin = FVector::ZeroVector;
		TArray<float> LaneYs;
		TArray<float> StepHeights;
		TArray<float> SpeedsMps;
		TArray<bool> bEnforcedModes;
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateSlopeScenario(const FString& Name)
{
	if (Name == TEXT("T4")) { return MakeUnique<FT4LateralSlopeScenario>(); }
	if (Name == TEXT("T5")) { return MakeUnique<FT5SlopeBrakeScenario>(); }
	if (Name == TEXT("T6")) { return MakeUnique<FT6StepScenario>(); }
	return nullptr;
}
