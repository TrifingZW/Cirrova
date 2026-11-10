#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
#pragma region T18 悬挂回退（Landscape 路径）

	/**
	 * T18：强制悬挂走射线弹簧回退（模拟 Landscape/非单粒子代理地面），
	 * 验证静置载荷、落地收敛与侧向斜坡保持，同时轮胎仍以求解器约束耦合。
	 */
	class FT18SuspensionFallbackScenario final : public FCirrovaTestScenario
	{
	public:
		FT18SuspensionFallbackScenario() : FCirrovaTestScenario(TEXT("T18"), 40.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Flat = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Flat);

			// v0 静置与 v1 0.5 m 落地共用平地。
			FCirrovaTestVehicleSetup FallbackSetup;
			FallbackSetup.bForceSuspensionFallback = true;
			Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Flat, 0.02f, -20.0f), FallbackSetup);
			Harness.SetVehicleLabel(0, TEXT("fallback_static"));
			Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Flat, 0.5f, 20.0f), FallbackSetup);
			Harness.SetVehicleLabel(1, TEXT("fallback_drop_0_5"));

			// v2 侧向斜坡（25°）。
			const FTransform Slope = Harness.MakeSlopedGround(
				Harness.GetTestOrigin() + FVector(0.0f, 6000.0f, 0.0f),
				25.0f,
				0.0f
			);
			Harness.SpawnGround(Slope);
			SlopeGround = Slope;
			Harness.SpawnVehicle(Harness.MakeVehicleOnGround(Slope, 0.0f), FallbackSetup);
			Harness.SetVehicleLabel(2, TEXT("fallback_slope_25"));
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (Elapsed < 8.0f) { return; }

			const double End = Harness.GetSamples(0).IsEmpty() ? 8.0 : Harness.GetSamples(0).Last().Time;

			// v0：静置。
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(0);
				if (!Samples.IsEmpty())
				{
					const FVehicleStat Stat = ComputeVehicleStat(Samples, End - 1.0, End, FVector::UpVector);
					AddCheck(Stat.bAllFinite, VehicleMetricName(0, TEXT("non_finite")));
					AddCheck(Stat.MeanSpeed < 0.1f, FString::Printf(TEXT("v0 静置平均速度 %.3f m/s"), Stat.MeanSpeed));
					const float Drift = static_cast<float>((Stat.EndLocation - Stat.StartLocation).Size2D()) * 0.01f;
					AddCheck(Drift < 0.05f, FString::Printf(TEXT("v0 静置漂移 %.3f m"), Drift));

					float LoadSum = 0.0f;
					float NativeSum = 0.0f;
					float ClampSum = 0.0f;
					for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
					{
						const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, End - 1.0, End);
						if (WheelStat.Count <= 0) { continue; }
						LoadSum += WheelStat.MeanLoad;
						NativeSum += WheelStat.NativeFraction;
						ClampSum += WheelStat.ClampFraction;
						AddCheck(WheelStat.ContactFraction > 0.98f,
							FString::Printf(TEXT("v0 车轮 %d 接地比例 %.3f"), WheelIndex, WheelStat.ContactFraction));
						AddCheck(WheelStat.AppliedFraction > 0.98f,
							FString::Printf(TEXT("v0 车轮 %d 轮胎约束启用比例 %.3f"), WheelIndex, WheelStat.AppliedFraction));
						AddCheck(WheelStat.MeanLoad > 2200.0f && WheelStat.MeanLoad < 2700.0f,
							FString::Printf(TEXT("v0 车轮 %d 平均载荷 %.1f N"), WheelIndex, WheelStat.MeanLoad));
					}
					AddCheck(NativeSum < 0.02f, FString::Printf(TEXT("v0 原生悬挂比例 %.3f 应接近 0"), NativeSum));
					AddCheck(ClampSum < 0.05f, FString::Printf(TEXT("v0 压缩速度钳制比例 %.3f"), ClampSum));
					AddCheck(LoadSum > 8800.0f && LoadSum < 10800.0f,
						FString::Printf(TEXT("v0 总法向载荷 %.0f N"), LoadSum));

					AddMetric(VehicleMetricName(0, TEXT("drift_m")), Drift);
					AddMetric(VehicleMetricName(0, TEXT("sum_load_n")), LoadSum);
					AddMetric(VehicleMetricName(0, TEXT("native_fraction")), NativeSum / 4.0f);
				}
			}

			// v1：0.5 m 落地收敛。
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(1);
				if (!Samples.IsEmpty())
				{
					const double DropEnd = Samples.Last().Time;
					const float ImpactTime = FirstContactTime(Samples);
					AddCheck(ImpactTime >= 0.0f, VehicleMetricName(1, TEXT("no_contact")));
					if (ImpactTime >= 0.0f)
					{
						const float SettledTime = FindSettledTime(Samples, ImpactTime, 0.15f, 1.0f);
						AddCheck(SettledTime >= 0.0f, VehicleMetricName(1, TEXT("not_settled")));
						if (SettledTime >= 0.0f)
						{
							AddCheck(SettledTime - ImpactTime < 3.5f,
								FString::Printf(TEXT("v1 收敛时间 %.2f s"), SettledTime - ImpactTime));
							AddMetric(VehicleMetricName(1, TEXT("settle_s")), SettledTime - ImpactTime);
						}
					}

					const FVehicleStat Stat = ComputeVehicleStat(Samples, 0.0, DropEnd, FVector::UpVector);
					AddCheck(Stat.bAllFinite, VehicleMetricName(1, TEXT("non_finite")));
					AddCheck(Stat.MaxTiltDegrees < 30.0f,
						FString::Printf(TEXT("v1 最大倾角 %.1f 度"), Stat.MaxTiltDegrees));

					float MaxCompression = 0.0f;
					float FinalLoad = 0.0f;
					int32 FinalCount = 0;
					for (const FCirrovaTestVehicleSample& Sample : Samples)
					{
						if (Sample.Time < DropEnd - 1.0) { continue; }
						for (const FCirrovaWheelState& Wheel : Sample.Wheels)
						{
							FinalLoad += Wheel.NormalLoad;
							++FinalCount;
						}
					}
					for (const FCirrovaTestVehicleSample& Sample : Samples)
					{
						for (const FCirrovaWheelState& Wheel : Sample.Wheels)
						{
							MaxCompression = FMath::Max(MaxCompression, Wheel.Compression);
						}
					}
					FinalLoad /= FMath::Max(static_cast<float>(FinalCount), 1.0f);
					AddCheck(MaxCompression < 0.18f, FString::Printf(TEXT("v1 峰值压缩 %.3f m"), MaxCompression));
					AddCheck(FinalLoad > 2200.0f && FinalLoad < 2700.0f,
						FString::Printf(TEXT("v1 末段平均每轮载荷 %.0f N"), FinalLoad));

					AddMetric(VehicleMetricName(1, TEXT("peak_compression_m")), MaxCompression);
					AddMetric(VehicleMetricName(1, TEXT("final_mean_load_n")), FinalLoad);
				}
			}

			// v2：25° 侧向斜坡静态保持。
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(2);
				if (!Samples.IsEmpty())
				{
					const double SlopeEnd = Samples.Last().Time;
					const FVector Normal = SlopeGround.GetUnitAxis(EAxis::Z);
					const FVector Right = SlopeGround.GetUnitAxis(EAxis::Y);
					const FVehicleStat Stat = ComputeVehicleStat(Samples, SlopeEnd - 2.0, SlopeEnd, Normal);
					AddCheck(Stat.bAllFinite, VehicleMetricName(2, TEXT("non_finite")));
					AddCheck(Stat.MinUpDot > 0.9f,
						FString::Printf(TEXT("v2 车身未贴合坡面，最小上向量点积 %.3f"), Stat.MinUpDot));

					const FVector Delta = (Stat.EndLocation - Stat.StartLocation) * 0.01f;
					const float DriftLateral = FMath::Abs(FVector::DotProduct(Delta, Right));
					AddCheck(DriftLateral < 0.6f, FString::Printf(TEXT("v2 侧向蠕滑 %.3f m"), DriftLateral));
					AddCheck(Stat.MeanSpeed < 0.6f,
						FString::Printf(TEXT("v2 测量窗平均速度 %.3f m/s"), Stat.MeanSpeed));

					const float ExpectedLoad = 1000.0f * Gravity * FMath::Cos(FMath::DegreesToRadians(25.0f));
					AddCheck(FMath::Abs(Stat.MeanSumLoad - ExpectedLoad) < ExpectedLoad * 0.15f,
						FString::Printf(TEXT("v2 总法向载荷 %.0f N（期望 %.0f）"), Stat.MeanSumLoad, ExpectedLoad));

					AddMetric(VehicleMetricName(2, TEXT("drift_lateral_m")), DriftLateral);
					AddMetric(VehicleMetricName(2, TEXT("mean_speed_mps")), Stat.MeanSpeed);
					AddMetric(VehicleMetricName(2, TEXT("sum_load_n")), Stat.MeanSumLoad);
				}
			}

			Complete(TEXT("悬挂射线弹簧回退在静置/落地/斜坡下稳定"));
		}

	private:
		FTransform SlopeGround = FTransform::Identity;
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateFallbackScenario(const FString& Name)
{
	if (Name == TEXT("T18")) { return MakeUnique<FT18SuspensionFallbackScenario>(); }
	return nullptr;
}
