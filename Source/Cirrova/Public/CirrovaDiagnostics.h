#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "CirrovaDiagnostics.generated.h"

/** Cirrova 运行时诊断：注册载具数量、轮胎约束槽与物理步耗时。 */
UCLASS()
class CIRROVA_API UCirrovaDiagnostics : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
#pragma region SPI

	// 当前已注册到调度器的载具数量。
	UFUNCTION(
		BlueprintPure,
		Category = "Cirrova载具组件|诊断",
		meta = (WorldContext = "WorldContextObject", DisplayName = "获取注册载具数量")
	)
	static int32 GetRegisteredVehicleCount(const UObject* WorldContextObject);

	// 当前仍绑定粒子的轮胎约束数量。
	UFUNCTION(
		BlueprintPure,
		Category = "Cirrova载具组件|诊断",
		meta = (WorldContext = "WorldContextObject", DisplayName = "获取活动轮胎约束数量")
	)
	static int32 GetActiveWheelConstraintCount(const UObject* WorldContextObject);

	// 最近一次物理步的墙钟耗时（ms）。
	UFUNCTION(
		BlueprintPure,
		Category = "Cirrova载具组件|诊断",
		meta = (WorldContext = "WorldContextObject", DisplayName = "获取最近物理步耗时 (ms)")
	)
	static float GetLastPhysicsStepMs(const UObject* WorldContextObject);

	// 自世界启动以来的物理步平均墙钟耗时（ms）。
	UFUNCTION(
		BlueprintPure,
		Category = "Cirrova载具组件|诊断",
		meta = (WorldContext = "WorldContextObject", DisplayName = "获取平均物理步耗时 (ms)")
	)
	static float GetAveragePhysicsStepMs(const UObject* WorldContextObject);

	// 自世界启动以来的物理步峰值墙钟耗时（ms）。
	UFUNCTION(
		BlueprintPure,
		Category = "Cirrova载具组件|诊断",
		meta = (WorldContext = "WorldContextObject", DisplayName = "获取峰值物理步耗时 (ms)")
	)
	static float GetPeakPhysicsStepMs(const UObject* WorldContextObject);

#pragma endregion
};
