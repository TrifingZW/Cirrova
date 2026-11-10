#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
	/** 车轮集合在窗口内的平均纵向力之和（N）。 */
	float SumMeanLongitudinalForce(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const TArray<int32>& WheelIndices,
		const double Start,
		const double End
	)
	{
		float Sum = 0.0f;
		for (const int32 WheelIndex : WheelIndices)
		{
			Sum += ComputeWheelStat(Samples, WheelIndex, Start, End).MeanLongitudinalForce;
		}
		return Sum;
	}

#pragma region T15 制动力分配

	/** T15：前/后制动比例 60/40、50/50、40/60，部分制动下验证逐轴扭矩分配。 */
	class FT15BrakeDistributionScenario final : public FCirrovaTestScenario
	{
	public:
		FT15BrakeDistributionScenario() : FCirrovaTestScenario(TEXT("T15"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			const float FrontScales[3] = {1.2f, 1.0f, 0.8f};
			const float RearScales[3] = {0.8f, 1.0f, 1.2f};
			const TCHAR* Labels[3] = {TEXT("bias_60_40"), TEXT("bias_50_50"), TEXT("bias_40_60")};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.FrontBrakeTorqueScale = FrontScales[Index];
				Setup.RearBrakeTorqueScale = RearScales[Index];
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
						// 部分制动：逐轴扭矩低于轮胎附着上限，力分配由制动容量决定。
						Vehicle->SetInputs(0.0f, 0.0f, 0.25f, 0);
						SetHeldVelocity(Vehicle, Vehicle->GetActorForwardVector(), 20.0f);
					}
				}
			}

			if (Elapsed < 7.0f) { return; }

			const TArray<int32> FrontWheels = {0, 1};
			const TArray<int32> RearWheels = {2, 3};
			const float ExpectedRatios[3] = {1.5f, 1.0f, 0.6667f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.7, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				const float FrontForce = SumMeanLongitudinalForce(Samples, FrontWheels, 2.6, 4.5);
				const float RearForce = SumMeanLongitudinalForce(Samples, RearWheels, 2.6, 4.5);
				AddCheck(FrontForce < -100.0f && RearForce < -100.0f,
					FString::Printf(TEXT("v%d 制动力不足：前 %.0f N / 后 %.0f N"), Index, FrontForce, RearForce));

				const float Ratio = RearForce < -1.0f ? FrontForce / RearForce : 0.0f;
				const float Expected = ExpectedRatios[Index];
				AddCheck(
					FMath::Abs(Ratio - Expected) < Expected * 0.25f,
					FString::Printf(
						TEXT("v%d 前后制动力比 %.2f（期望 %.2f）"),
						Index,
						Ratio,
						Expected
					)
				);

				const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(Index);
				AddCheck(GetSpeedKmh(Latest) / 3.6f < 1.0f,
					FString::Printf(TEXT("v%d 最终速度 %.2f m/s"), Index, GetSpeedKmh(Latest) / 3.6f));

				AddMetric(VehicleMetricName(Index, TEXT("front_force_n")), FrontForce);
				AddMetric(VehicleMetricName(Index, TEXT("rear_force_n")), RearForce);
				AddMetric(VehicleMetricName(Index, TEXT("force_ratio")), Ratio);
			}

			Complete(TEXT("前后轴制动力按配置比例分配"));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion

#pragma region T16 ABS

	/** T16：低附着路面 20 m/s 全制动，对比 ABS 开/关的锁止与停止距离。 */
	class FT16AbsScenario final : public FCirrovaTestScenario
	{
	public:
		FT16AbsScenario() : FCirrovaTestScenario(TEXT("T16"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			// 低附着地面：动摩擦 0.35；扩大范围以容纳 20 m/s 全制动距离。
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin(), 250.0f);
			Harness.SpawnGround(Ground, 0.35f, 0.42f);

			const bool AbsStates[2] = {false, true};
			const TCHAR* Labels[2] = {TEXT("abs_off"), TEXT("abs_on")};
			for (int32 Index = 0; Index < 2; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.bEnableABS = AbsStates[Index];
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(Ground, 0.02f, 0.0f, (Index - 0.5f) * 40.0f),
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
				for (int32 Index = 0; Index < 2; ++Index)
				{
					if (ACirrovaTestVehicle* Vehicle = Harness.GetVehicle(Index))
					{
						Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0);
						SetHeldVelocity(Vehicle, Vehicle->GetActorForwardVector(), 20.0f);
					}
				}
			}

			if (Elapsed < 10.0f) { return; }

			float StopDistances[2] = {0.0f, 0.0f};
			float PeakSlips[2] = {0.0f, 0.0f};
			float LockFractions[2] = {0.0f, 0.0f};
			float MinModulations[2] = {1.0f, 1.0f};
			for (int32 Index = 0; Index < 2; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.6, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));

				StopDistances[Index] = static_cast<float>(
					(Stat.EndLocation - Stat.StartLocation).Size2D()) * 0.01f;

				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 2.6, 6.5);
					if (WheelStat.Count <= 0) { continue; }
					PeakSlips[Index] = FMath::Max(PeakSlips[Index], WheelStat.MeanAbsSlip);
					LockFractions[Index] = FMath::Max(LockFractions[Index], WheelStat.LockedFraction);
				}
				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					if (Sample.Time < 2.6) { continue; }
					for (const FCirrovaWheelState& Wheel : Sample.Wheels)
					{
						MinModulations[Index] = FMath::Min(MinModulations[Index], Wheel.ABSModulation);
					}
				}

				const FCirrovaTestVehicleSample* Latest = Harness.GetLatest(Index);
				AddCheck(GetSpeedKmh(Latest) / 3.6f < 0.8f,
					FString::Printf(TEXT("v%d 最终速度 %.2f m/s"), Index, GetSpeedKmh(Latest) / 3.6f));
			}

			// ABS 关：车轮长时间锁止拖滑；ABS 开：泄压后滑移回落，锁止帧占比显著更低。
			AddCheck(LockFractions[0] > 0.5f,
				FString::Printf(TEXT("ABS 关车辆锁止帧占比 %.2f 过低"), LockFractions[0]));
			AddCheck(LockFractions[1] < LockFractions[0] - 0.2f,
				FString::Printf(
					TEXT("ABS 开锁止帧占比 %.2f 未显著低于关 %.2f"),
					LockFractions[1],
					LockFractions[0]
				));
			AddCheck(MinModulations[1] < 0.9f,
				FString::Printf(TEXT("ABS 开未见泄压（最小调制 %.2f）"), MinModulations[1]));
			AddCheck(StopDistances[1] <= StopDistances[0] * 1.05f + 1.0f,
				FString::Printf(
					TEXT("ABS 开停止距离 %.1f m 明显长于关 %.1f m"),
					StopDistances[1],
					StopDistances[0]
				));

			AddMetric(TEXT("abs_off_stop_m"), StopDistances[0]);
			AddMetric(TEXT("abs_on_stop_m"), StopDistances[1]);
			AddMetric(TEXT("abs_off_lock_fraction"), LockFractions[0]);
			AddMetric(TEXT("abs_on_lock_fraction"), LockFractions[1]);
			AddMetric(TEXT("abs_on_min_modulation"), MinModulations[1]);
			AddMetric(TEXT("abs_off_peak_slip"), PeakSlips[0]);
			AddMetric(TEXT("abs_on_peak_slip"), PeakSlips[1]);

			Complete(TEXT("ABS 在低附着路面降低锁止与滑移"));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion

#pragma region T17 限滑参数化

	/** T17：对分附着路面静止起步，验证差速器限滑扭矩容量对驱动力分配的影响。 */
	class FT17LimitedSlipScenario final : public FCirrovaTestScenario
	{
	public:
		FT17LimitedSlipScenario() : FCirrovaTestScenario(TEXT("T17"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			// 对分路面：左侧（Y−）动摩擦 1.0，右侧（Y+）动摩擦 0.12（低于单轮驱动力上限的一半，
			// 使开式差速下驱动扭矩受低附着侧拖累，限滑容量才是控制量）；接缝沿车身纵向中面。
			// 每车独立地面，避免不同限滑参数下的加速差异导致追尾。
			constexpr float ExtentM = 60.0f;
			const FVector Origin = Harness.GetTestOrigin();

			const float LsdTorques[3] = {0.0f, 50.0f, 100.0f};
			const TCHAR* Labels[3] = {TEXT("lsd_0"), TEXT("lsd_50"), TEXT("lsd_100")};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const FVector PairCenter = Origin + FVector((Index - 1) * 18000.0f, 0.0f, 0.0f);
				const FTransform HighSide = Harness.MakeFlatGround(
					PairCenter - FVector(0.0f, ExtentM * 100.0f, 0.0f),
					ExtentM
				);
				const FTransform LowSide = Harness.MakeFlatGround(
					PairCenter + FVector(0.0f, ExtentM * 100.0f, 0.0f),
					ExtentM
				);
				Harness.SpawnGround(HighSide, 1.0f, 1.0f);
				Harness.SpawnGround(LowSide, 0.12f, 0.15f);

				FCirrovaTestVehicleSetup Setup;
				Setup.DifferentialMaxTorqueOverride = LsdTorques[Index];
				// 车辆放在高侧地面上，横向偏移使轮胎分居接缝两侧。
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(HighSide, 0.02f, 0.0f, ExtentM),
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

			if (Elapsed < 6.5f) { return; }

			float Distances[3] = {0.0f, 0.0f, 0.0f};
			float HighSideForces[3] = {0.0f, 0.0f, 0.0f};
			for (int32 Index = 0; Index < 3; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat Stat = ComputeVehicleStat(Samples, 2.7, End, FVector::UpVector);
				AddCheck(Stat.bAllFinite, VehicleMetricName(Index, TEXT("non_finite")));
				Distances[Index] = static_cast<float>(
					(Stat.EndLocation - Stat.StartLocation).Size2D()) * 0.01f;

				// 高附着侧车轮由遥测地面摩擦系数识别（≥0.6）。
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, 2.7, 6.0);
					if (WheelStat.Count > 0 && WheelStat.MeanGroundFriction >= 0.6f)
					{
						HighSideForces[Index] += WheelStat.MeanLongitudinalForce;
					}
				}

				AddCheck(Distances[Index] > 0.5f,
					FString::Printf(TEXT("v%d 行驶距离 %.2f m 过小"), Index, Distances[Index]));
			}

			// 限滑容量越大，高附着侧获得的驱动力与整车推进越强。
			AddCheck(
				HighSideForces[1] > HighSideForces[0] - 50.0f
					&& HighSideForces[2] > HighSideForces[1] - 50.0f,
				FString::Printf(
					TEXT("高附着侧驱动力未随限滑扭矩单调（%.0f / %.0f / %.0f N）"),
					HighSideForces[0],
					HighSideForces[1],
					HighSideForces[2]
				)
			);
			AddCheck(
				Distances[1] > Distances[0] - 0.2f && Distances[2] > Distances[1] - 0.2f,
				FString::Printf(
					TEXT("行驶距离未随限滑扭矩单调（%.2f / %.2f / %.2f m）"),
					Distances[0],
					Distances[1],
					Distances[2]
				)
			);

			for (int32 Index = 0; Index < 3; ++Index)
			{
				AddMetric(VehicleMetricName(Index, TEXT("distance_m")), Distances[Index]);
				AddMetric(VehicleMetricName(Index, TEXT("high_side_force_n")), HighSideForces[Index]);
			}

			Complete(TEXT("限滑扭矩容量按序分配对分路面驱动力"));
		}

	private:
		bool bLaunched = false;
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateBrakingScenario(const FString& Name)
{
	if (Name == TEXT("T15")) { return MakeUnique<FT15BrakeDistributionScenario>(); }
	if (Name == TEXT("T16")) { return MakeUnique<FT16AbsScenario>(); }
	if (Name == TEXT("T17")) { return MakeUnique<FT17LimitedSlipScenario>(); }
	return nullptr;
}
