#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleComponent.h"
#include "CirrovaTestVehicle.h"

class ACirrovaTestGround;
class ACirrovaTestVehicle;
class FArchive;

/** 单载具的单帧测试采样。 */
struct FCirrovaTestVehicleSample
{
	double Time = 0.0;
	FVector Location = FVector::ZeroVector;
	FVector Velocity = FVector::ZeroVector;
	FVector AngularVelocityDegrees = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	bool bGrounded = false;
	float EngineRPM = 0.0f;
	bool bTireCoupling = false;
	int32 AppliedSubsteps = 1;
	float SolverDeltaTime = 0.0f;
	TArray<FCirrovaWheelState> Wheels;

	// 遥测与位姿全部为有限值。
	bool IsFinite() const;
};

/** 场景指标。 */
struct FCirrovaTestMetric
{
	FString Name;
	double Value = 0.0;
};

/** 单场景结果。 */
struct FCirrovaTestScenarioResult
{
	FString Name;
	bool bPassed = false;
	float Duration = 0.0f;
	FString Summary;
	TArray<FString> Failures;
	TArray<FCirrovaTestMetric> Metrics;
};

/**
 * P0 自动化测试运行器：创建受控地面与载具，逐帧采样并输出 CSV/JSON。
 *
 * 采样使用游戏线程遥测快照；物理步进本身由 Chaos 与调度器驱动。
 */
class FCirrovaTestHarness final
{
public:
	FCirrovaTestHarness(UWorld* InWorld, FString InOutputDir);
	~FCirrovaTestHarness();

	FCirrovaTestHarness(const FCirrovaTestHarness&) = delete;
	FCirrovaTestHarness& operator=(const FCirrovaTestHarness&) = delete;

	UWorld* GetWorld() const { return World; }
	double GetTime() const { return Time; }
	const FString& GetOutputDir() const { return OutputDir; }

	// 创建受控几何与载具；创建顺序即车辆索引。
	ACirrovaTestGround* SpawnGround(
		const FTransform& Transform,
		float Friction = 1.0f,
		float StaticFriction = 1.0f
	);
	ACirrovaTestVehicle* SpawnVehicle(const FTransform& Transform, const FCirrovaTestVehicleSetup& Setup);
	void DestroySpawned();

	int32 GetVehicleCount() const { return Vehicles.Num(); }
	ACirrovaTestVehicle* GetVehicle(int32 Index) const;

	// 每辆车对应的子用例标签，用于 CSV 与结果说明。
	void SetVehicleLabel(int32 Index, const FString& Label);
	const FString& GetVehicleLabel(int32 Index) const;

	// 逐帧采样；由世界子系统在场景 Tick 前调用。
	void Tick(float DeltaTime);

	const TArray<FCirrovaTestVehicleSample>& GetSamples(int32 Index) const;
	const FCirrovaTestVehicleSample* GetLatest(int32 Index) const;

	// CSV 生命周期。
	void BeginCase(const FString& CaseName);
	void EndCase();

	// 汇总报告。
	void WriteReport(const TArray<FCirrovaTestScenarioResult>& Results, bool bAllPassed) const;

	// 调度器诊断。
	int32 GetRegisteredVehicleCount() const;
	int32 GetActiveWheelConstraintCount() const;
	float GetLastStepMs() const;
	float GetAverageStepMs() const;
	float GetPeakStepMs() const;
	const TArray<float>& GetPhysicsStepMs() const { return PhysicsStepMs; }

	// 场景几何：地面顶面通过 Center，法线为旋转后的 Z 轴。
	static FVector GetTestOrigin();
	static FTransform MakeFlatGround(const FVector& Center, float ExtentM = 100.0f);
	static FTransform MakeSlopedGround(const FVector& Center, float RollDegrees, float PitchDegrees);
	static FTransform MakeBox(const FVector& Center, const FVector& FullSizeCm, float YawDegrees = 0.0f);

	// 车轮底面位于地面顶面以上 WheelBottomHeightM 处。
	static FTransform MakeVehicleOnGround(
		const FTransform& Ground,
		float WheelBottomHeightM,
		float ForwardOffsetM = 0.0f,
		float LateralOffsetM = 0.0f,
		float YawOffsetDegrees = 0.0f
	);

	static constexpr float WheelRadiusM = 0.35f;
	static constexpr float SuspensionRestLengthM = 0.35f;

private:
	bool EnsureCsvOpen() const;

	UWorld* World = nullptr;
	FString OutputDir;

	TArray<TObjectPtr<AActor>> SpawnedActors;
	TArray<TObjectPtr<ACirrovaTestVehicle>> Vehicles;
	TArray<FString> VehicleLabels;
	TArray<TArray<FCirrovaTestVehicleSample>> Samples;
	TArray<float> PhysicsStepMs;

	FString CaseName;
	TUniquePtr<FArchive> CsvFile;
	int32 CsvRowsSinceFlush = 0;
	double Time = 0.0;
};
