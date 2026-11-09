#include "CirrovaTestScenarios.h"

#include "CirrovaTestSupport.h"

using namespace CirrovaTestSupport;

namespace
{
#pragma region T11 悬挂模型

	/** T11：对比基线悬挂与 P1.2 分向阻尼/刚度曲线/渐进限位/预载。 */
	class FT11SuspensionModelScenario final : public FCirrovaTestScenario
	{
	public:
		FT11SuspensionModelScenario() : FCirrovaTestScenario(TEXT("T11"), 30.0f) {}

	protected:
		void OnBegin(FCirrovaTestHarness& Harness) override
		{
			const FTransform Ground = Harness.MakeFlatGround(Harness.GetTestOrigin());
			Harness.SpawnGround(Ground);

			struct FCase
			{
				float DropHeightM = 0.5f;
				float OffsetX = 0.0f;
				bool bBaseline = false;
				bool bProgressive = false;
				const TCHAR* Label = TEXT("");
			};
			const FCase Cases[4] = {
				{0.5f, -30.0f, true, false, TEXT("base_0_5")},
				{0.5f, -10.0f, false, true, TEXT("p12_0_5")},
				{2.0f, 10.0f, true, false, TEXT("base_2_0")},
				{2.0f, 30.0f, false, true, TEXT("p12_2_0")}
			};

			for (int32 Index = 0; Index < 4; ++Index)
			{
				FCirrovaTestVehicleSetup Setup;
				Setup.bUseBaselineSuspension = Cases[Index].bBaseline;
				Setup.bUseProgressiveSuspension = Cases[Index].bProgressive;
				ACirrovaTestVehicle* Vehicle = Harness.SpawnVehicle(
					Harness.MakeVehicleOnGround(
						Ground,
						Cases[Index].DropHeightM,
						Cases[Index].OffsetX
					),
					Setup
				);
				if (Vehicle) { Vehicle->SetInputs(0.0f, 0.0f, 1.0f, 0); }
				Harness.SetVehicleLabel(Index, Cases[Index].Label);
			}
		}

		void OnTick(FCirrovaTestHarness& Harness, float) override
		{
			if (Elapsed < 7.0f) { return; }

			struct FSuspensionStats
			{
				float MaxCompression = 0.0f;
				float MaxLoad = 0.0f;
				float MaxReboundSpeed = 0.0f;
				float FinalCompression = 0.0f;
				float FinalLoad = 0.0f;
				float SettleTime = -1.0f;
				float MaxTilt = 0.0f;
				bool bFinite = true;
			};

			FSuspensionStats Stats[4];
			for (int32 Index = 0; Index < 4; ++Index)
			{
				const TArray<FCirrovaTestVehicleSample>& Samples = Harness.GetSamples(Index);
				if (Samples.IsEmpty()) { continue; }

				const double End = Samples.Last().Time;
				const FVehicleStat AllStat = ComputeVehicleStat(Samples, 0.0, End, FVector::UpVector);
				Stats[Index].bFinite = AllStat.bAllFinite;
				Stats[Index].MaxTilt = AllStat.MaxTiltDegrees;

				const float ImpactTime = FirstContactTime(Samples);
				if (ImpactTime >= 0.0f)
				{
					const float SettledTime = FindSettledTime(Samples, ImpactTime, 0.15f, 1.0f);
					if (SettledTime >= 0.0f) { Stats[Index].SettleTime = SettledTime - ImpactTime; }
				}

				for (const FCirrovaTestVehicleSample& Sample : Samples)
				{
					for (const FCirrovaWheelState& Wheel : Sample.Wheels)
					{
						Stats[Index].MaxCompression = FMath::Max(Stats[Index].MaxCompression, Wheel.Compression);
						Stats[Index].MaxLoad = FMath::Max(Stats[Index].MaxLoad, Wheel.NormalLoad);
						Stats[Index].MaxReboundSpeed = FMath::Max(
							Stats[Index].MaxReboundSpeed,
							FMath::Max(-Wheel.CompressionVelocity, 0.0f)
						);
					}
				}

				float CompressionSum = 0.0f;
				float LoadSum = 0.0f;
				for (int32 WheelIndex = 0; WheelIndex < 4; ++WheelIndex)
				{
					const FWheelStat WheelStat = ComputeWheelStat(Samples, WheelIndex, End - 1.0, End);
					CompressionSum += WheelStat.MeanCompression;
					LoadSum += WheelStat.MeanLoad;
				}
				Stats[Index].FinalCompression = CompressionSum / 4.0f;
				Stats[Index].FinalLoad = LoadSum / 4.0f;

				AddCheck(Stats[Index].bFinite, VehicleMetricName(Index, TEXT("non_finite")));
				AddCheck(Stats[Index].SettleTime >= 0.0f && Stats[Index].SettleTime < 3.5f, FString::Printf(TEXT("v%d 收敛时间 %.2f s"), Index, Stats[Index].SettleTime));
				AddCheck(Stats[Index].MaxTilt < 30.0f, FString::Printf(TEXT("v%d 最大倾角 %.1f 度"), Index, Stats[Index].MaxTilt));
				AddCheck(Stats[Index].FinalLoad > 2200.0f && Stats[Index].FinalLoad < 2700.0f, FString::Printf(TEXT("v%d 最终载荷 %.0f N"), Index, Stats[Index].FinalLoad));

				AddMetric(VehicleMetricName(Index, TEXT("max_compression_m")), Stats[Index].MaxCompression);
				AddMetric(VehicleMetricName(Index, TEXT("max_load_n")), Stats[Index].MaxLoad);
				AddMetric(VehicleMetricName(Index, TEXT("max_rebound_mps")), Stats[Index].MaxReboundSpeed);
				AddMetric(VehicleMetricName(Index, TEXT("final_compression_m")), Stats[Index].FinalCompression);
				AddMetric(VehicleMetricName(Index, TEXT("final_load_n")), Stats[Index].FinalLoad);
				AddMetric(VehicleMetricName(Index, TEXT("settle_s")), Stats[Index].SettleTime);
			}

			// 0.5 m：预载降低静态压缩；刚度曲线/渐进限位提高峰值载荷；分向回弹阻尼限制回弹速度。
			AddCheck(Stats[1].FinalCompression < Stats[0].FinalCompression + 0.001f, TEXT("P1.2 预载未降低 0.5 m 落地的静态压缩"));
			AddCheck(Stats[1].MaxCompression < Stats[0].MaxCompression + 0.001f, TEXT("P1.2 刚度曲线未限制 0.5 m 落地压缩"));
			AddCheck(Stats[1].MaxLoad > Stats[0].MaxLoad, TEXT("P1.2 刚度曲线/预载未提高 0.5 m 落地峰值载荷"));
			AddCheck(Stats[1].MaxReboundSpeed < Stats[0].MaxReboundSpeed * 1.3f + 0.05f, TEXT("P1.2 回弹阻尼未限制 0.5 m 落地回弹速度"));

			// 2.0 m：两侧均应稳定且峰值有界；渐进限位在 0.12–0.15 m 行程附近由 T1/T6 覆盖。
			AddCheck(Stats[3].MaxLoad < 70000.0f, FString::Printf(TEXT("P1.2 2.0 m 落地峰值载荷 %.0f N 过大"), Stats[3].MaxLoad));

			Complete(TEXT("P1.2 分向阻尼/刚度曲线/渐进限位/预载行为符合预期"));
		}
	};

#pragma endregion
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateSuspensionScenario(const FString& Name)
{
	if (Name == TEXT("T11")) { return MakeUnique<FT11SuspensionModelScenario>(); }
	return nullptr;
}
