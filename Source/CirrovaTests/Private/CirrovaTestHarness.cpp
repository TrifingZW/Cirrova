#include "CirrovaTestHarness.h"

#include "CirrovaDiagnostics.h"
#include "CirrovaLog.h"
#include "CirrovaTestGround.h"
#include "CirrovaTestVehicle.h"
#include "Components/PrimitiveComponent.h"
#include "Containers/StringConv.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"
#include "Policies/PrettyJsonPrintPolicy.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
	bool IsFiniteVector(const FVector& Vector)
	{
		return FMath::IsFinite(Vector.X) && FMath::IsFinite(Vector.Y) && FMath::IsFinite(Vector.Z);
	}

	bool IsFiniteQuat(const FQuat& Quat)
	{
		return FMath::IsFinite(Quat.X) && FMath::IsFinite(Quat.Y)
			&& FMath::IsFinite(Quat.Z) && FMath::IsFinite(Quat.W);
	}

	bool IsFiniteWheel(const FCirrovaWheelState& Wheel)
	{
		return FMath::IsFinite(Wheel.Omega)
			&& FMath::IsFinite(Wheel.SurfaceSpeed)
			&& FMath::IsFinite(Wheel.EffectiveInertia)
			&& FMath::IsFinite(Wheel.ConstraintWheelInverseMass)
			&& FMath::IsFinite(Wheel.SuspensionLength)
			&& FMath::IsFinite(Wheel.Compression)
			&& FMath::IsFinite(Wheel.CompressionVelocity)
			&& FMath::IsFinite(Wheel.SuspensionForce)
			&& FMath::IsFinite(Wheel.AntiRollForce)
			&& FMath::IsFinite(Wheel.NormalLoad)
			&& FMath::IsFinite(Wheel.TireLongitudinalForce)
			&& FMath::IsFinite(Wheel.TireLateralForce)
			&& FMath::IsFinite(Wheel.WheelTorque)
			&& FMath::IsFinite(Wheel.RollingResistance)
			&& FMath::IsFinite(Wheel.LongitudinalImpulse)
			&& FMath::IsFinite(Wheel.LateralImpulse)
			&& FMath::IsFinite(Wheel.ImpulseWindow)
			&& FMath::IsFinite(Wheel.ContactLongitudinalSpeed)
			&& FMath::IsFinite(Wheel.ContactLateralSpeed)
			&& FMath::IsFinite(Wheel.FrictionForceLimit)
			&& FMath::IsFinite(Wheel.LateralForceLimit)
			&& FMath::IsFinite(Wheel.StaticFrictionForceLimit)
			&& FMath::IsFinite(Wheel.StaticLateralForceLimit)
			&& FMath::IsFinite(Wheel.KineticFrictionForceLimit)
			&& FMath::IsFinite(Wheel.KineticLateralForceLimit)
			&& FMath::IsFinite(Wheel.GroundFriction)
			&& FMath::IsFinite(Wheel.GroundStaticFriction)
			&& FMath::IsFinite(Wheel.RelaxedSurfaceSpeed)
			&& FMath::IsFinite(Wheel.FrictionUtilization)
			&& FMath::IsFinite(Wheel.SlipRatio)
			&& FMath::IsFinite(Wheel.SlipAngleDegrees)
			&& FMath::IsFinite(Wheel.RelaxedSlipAngleDegrees)
			&& IsFiniteVector(Wheel.ContactPoint)
			&& IsFiniteVector(Wheel.ContactNormal)
			&& IsFiniteVector(Wheel.LongitudinalAxis)
			&& IsFiniteVector(Wheel.LateralAxis)
			&& IsFiniteVector(Wheel.ContactVelocity)
			&& IsFiniteVector(Wheel.GroundVelocity);
	}

	FString SanitizeFileStem(const FString& Name)
	{
		FString Result = Name;
		for (TCHAR& Character : Result)
		{
			if (!FChar::IsAlnum(Character) && Character != TEXT('_') && Character != TEXT('-'))
			{
				Character = TEXT('_');
			}
		}
		return Result.IsEmpty() ? TEXT("case") : Result;
	}

	FString BoolText(const bool bValue) { return bValue ? TEXT("1") : TEXT("0"); }

	void AppendFloat(TArray<FString>& Fields, const float Value)
	{
		Fields.Add(FString::Printf(TEXT("%.6f"), Value));
	}
}

bool FCirrovaTestVehicleSample::IsFinite() const
{
	if (!IsFiniteVector(Location) || !IsFiniteVector(Velocity)
		|| !IsFiniteVector(AngularVelocityDegrees) || !IsFiniteQuat(Rotation)
		|| !FMath::IsFinite(EngineRPM))
	{
		return false;
	}

	for (const FCirrovaWheelState& Wheel : Wheels)
	{
		if (!IsFiniteWheel(Wheel)) { return false; }
	}
	return true;
}

FCirrovaTestHarness::FCirrovaTestHarness(UWorld* InWorld, FString InOutputDir)
	: World(InWorld)
	, OutputDir(MoveTemp(InOutputDir))
{
	IFileManager::Get().MakeDirectory(*OutputDir, true);
}

FCirrovaTestHarness::~FCirrovaTestHarness() { EndCase(); }

ACirrovaTestGround* FCirrovaTestHarness::SpawnGround(
	const FTransform& Transform,
	const float Friction,
	const float StaticFriction
)
{
	if (!World) { return nullptr; }

	ACirrovaTestGround* Ground = World->SpawnActorDeferred<ACirrovaTestGround>(
		ACirrovaTestGround::StaticClass(),
		Transform,
		nullptr,
		nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn
	);
	if (!Ground) { return nullptr; }

	Ground->ConfigureSurface(Friction, StaticFriction);
	Ground->FinishSpawning(Transform);
	SpawnedActors.Add(Ground);
	return Ground;
}

ACirrovaTestVehicle* FCirrovaTestHarness::SpawnVehicle(
	const FTransform& Transform,
	const FCirrovaTestVehicleSetup& Setup
)
{
	if (!World) { return nullptr; }

	ACirrovaTestVehicle* NewVehicle = World->SpawnActorDeferred<ACirrovaTestVehicle>(
		ACirrovaTestVehicle::StaticClass(),
		Transform,
		nullptr,
		nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn
	);
	if (!NewVehicle) { return nullptr; }

	NewVehicle->Configure(Setup);
	NewVehicle->FinishSpawning(Transform);

	SpawnedActors.Add(NewVehicle);
	const int32 Index = Vehicles.Add(NewVehicle);
	Samples.AddDefaulted();
	VehicleLabels.AddDefaulted();
	VehicleLabels[Index] = FString::Printf(TEXT("vehicle_%d"), Index);
	return NewVehicle;
}

void FCirrovaTestHarness::DestroySpawned()
{
	for (AActor* Actor : SpawnedActors)
	{
		if (IsValid(Actor)) { Actor->Destroy(); }
	}

	SpawnedActors.Reset();
	Vehicles.Reset();
	VehicleLabels.Reset();
	Samples.Reset();
	PhysicsStepMs.Reset();
}

ACirrovaTestVehicle* FCirrovaTestHarness::GetVehicle(const int32 Index) const
{
	return Vehicles.IsValidIndex(Index) ? Vehicles[Index].Get() : nullptr;
}

void FCirrovaTestHarness::SetVehicleLabel(const int32 Index, const FString& Label)
{
	if (VehicleLabels.IsValidIndex(Index)) { VehicleLabels[Index] = Label; }
}

const FString& FCirrovaTestHarness::GetVehicleLabel(const int32 Index) const
{
	static const FString Empty;
	return VehicleLabels.IsValidIndex(Index) ? VehicleLabels[Index] : Empty;
}

void FCirrovaTestHarness::Tick(const float DeltaTime)
{
	Time += DeltaTime;
	PhysicsStepMs.Add(GetLastStepMs());

	if (Samples.Num() != Vehicles.Num())
	{
		Samples.SetNum(Vehicles.Num());
		VehicleLabels.SetNum(Vehicles.Num());
	}

	const FString ActiveCase = CaseName;
	for (int32 Index = 0; Index < Vehicles.Num(); ++Index)
	{
		ACirrovaTestVehicle* Vehicle = Vehicles[Index].Get();
		if (!IsValid(Vehicle)) { continue; }

		Vehicle->KeepAwake();

		const UCirrovaVehicleComponent* Component = Vehicle->GetVehicleComponent();
		const UPrimitiveComponent* Chassis = Vehicle->GetChassis();
		if (!Component || !Chassis) { continue; }

		FCirrovaTestVehicleSample Sample;
		Sample.Time = Time;
		Sample.Location = Vehicle->GetActorLocation();
		Sample.Velocity = Chassis->GetPhysicsLinearVelocity();
		Sample.AngularVelocityDegrees = Chassis->GetPhysicsAngularVelocityInDegrees();
		Sample.Rotation = Vehicle->GetActorQuat();
		Sample.bGrounded = Component->IsGrounded();
		Sample.EngineRPM = Component->GetEngineRPM();
		Sample.bTireCoupling = Component->IsTireConstraintCoupling();
		Sample.AppliedSubsteps = Component->GetAppliedSubsteps();
		Sample.SolverDeltaTime = Component->GetSolverDeltaTime();
		Sample.Wheels = Component->GetWheelStates();

		Samples[Index].Add(MoveTemp(Sample));
		const FCirrovaTestVehicleSample& Latest = Samples[Index].Last();

		if (!EnsureCsvOpen()) { continue; }

		const FString& Label = GetVehicleLabel(Index);
		const int32 ActiveConstraints = GetActiveWheelConstraintCount();
		const int32 RegisteredVehicles = GetRegisteredVehicleCount();

		TArray<FString> PoseFields;
		PoseFields.Reserve(24);
		PoseFields.Add(ActiveCase);
		PoseFields.Add(Label);
		PoseFields.Add(FString::Printf(TEXT("%d"), Index));
		AppendFloat(PoseFields, GetLastStepMs());
		PoseFields.Add(FString::Printf(TEXT("%d"), ActiveConstraints));
		PoseFields.Add(FString::Printf(TEXT("%d"), RegisteredVehicles));
		AppendFloat(PoseFields, Latest.Location.X);
		AppendFloat(PoseFields, Latest.Location.Y);
		AppendFloat(PoseFields, Latest.Location.Z);
		AppendFloat(PoseFields, Latest.Velocity.X);
		AppendFloat(PoseFields, Latest.Velocity.Y);
		AppendFloat(PoseFields, Latest.Velocity.Z);
		AppendFloat(PoseFields, Latest.AngularVelocityDegrees.X);
		AppendFloat(PoseFields, Latest.AngularVelocityDegrees.Y);
		AppendFloat(PoseFields, Latest.AngularVelocityDegrees.Z);
		PoseFields.Add(FString::Printf(TEXT("%.4f"), Latest.Rotation.X));
		PoseFields.Add(FString::Printf(TEXT("%.4f"), Latest.Rotation.Y));
		PoseFields.Add(FString::Printf(TEXT("%.4f"), Latest.Rotation.Z));
		PoseFields.Add(FString::Printf(TEXT("%.4f"), Latest.Rotation.W));
		PoseFields.Add(BoolText(Latest.bGrounded));
		AppendFloat(PoseFields, Latest.EngineRPM);
		PoseFields.Add(BoolText(Latest.bTireCoupling));
		PoseFields.Add(FString::Printf(TEXT("%d"), Latest.AppliedSubsteps));
		AppendFloat(PoseFields, Latest.SolverDeltaTime * 1000.0f);
		const FString Pose = FString::Join(PoseFields, TEXT(","));

		if (Latest.Wheels.IsEmpty()) { continue; }

		for (int32 WheelIndex = 0; WheelIndex < Latest.Wheels.Num(); ++WheelIndex)
		{
			const FCirrovaWheelState& Wheel = Latest.Wheels[WheelIndex];
			TArray<FString> Fields;
			Fields.Reserve(3 + 22 + 35);
			Fields.Add(FString::Printf(TEXT("%.4f"), Time));
			Fields.Add(Pose);
			Fields.Add(FString::Printf(TEXT("%d"), WheelIndex));
			Fields.Add(BoolText(Wheel.bInContact));
			AppendFloat(Fields, Wheel.NormalLoad);
			AppendFloat(Fields, Wheel.SuspensionLength);
			AppendFloat(Fields, Wheel.Compression);
			AppendFloat(Fields, Wheel.CompressionVelocity);
			Fields.Add(BoolText(Wheel.bCompressionVelocityClamped));
			AppendFloat(Fields, Wheel.SlipRatio);
			AppendFloat(Fields, Wheel.SlipAngleDegrees);
			AppendFloat(Fields, Wheel.RelaxedSlipAngleDegrees);
			AppendFloat(Fields, Wheel.TireLongitudinalForce);
			AppendFloat(Fields, Wheel.TireLateralForce);
			AppendFloat(Fields, Wheel.Omega);
			AppendFloat(Fields, Wheel.SurfaceSpeed);
			AppendFloat(Fields, Wheel.EffectiveInertia);
			AppendFloat(Fields, Wheel.FrictionForceLimit);
			AppendFloat(Fields, Wheel.LateralForceLimit);
			AppendFloat(Fields, Wheel.FrictionUtilization);
			Fields.Add(BoolText(Wheel.bNativeSuspensionActive));
			Fields.Add(BoolText(Wheel.bUsingConstraintLength));
			AppendFloat(Fields, Wheel.SuspensionConstraintLength);
			AppendFloat(Fields, Wheel.AntiRollForce);
			AppendFloat(Fields, Wheel.RollingResistance);
			Fields.Add(BoolText(Wheel.bTireConstraintApplied));
			AppendFloat(Fields, Wheel.LongitudinalImpulse);
			AppendFloat(Fields, Wheel.LateralImpulse);
			AppendFloat(Fields, Wheel.ImpulseWindow);
			AppendFloat(Fields, Wheel.ConstraintWheelInverseMass);
			AppendFloat(Fields, Wheel.StaticFrictionForceLimit);
			AppendFloat(Fields, Wheel.StaticLateralForceLimit);
			AppendFloat(Fields, Wheel.KineticFrictionForceLimit);
			AppendFloat(Fields, Wheel.KineticLateralForceLimit);
			AppendFloat(Fields, Wheel.GroundFriction);
			AppendFloat(Fields, Wheel.GroundStaticFriction);
			AppendFloat(Fields, Wheel.RelaxedSurfaceSpeed);

			const FString Line = FString::Join(Fields, TEXT(",")) + TEXT("\r\n");
			FTCHARToUTF8 Utf8(*Line);
			CsvFile->Serialize(reinterpret_cast<uint8*>(const_cast<ANSICHAR*>(Utf8.Get())), Utf8.Length());
			++CsvRowsSinceFlush;
		}
	}

	if (CsvFile && CsvRowsSinceFlush >= 120)
	{
		CsvFile->Flush();
		CsvRowsSinceFlush = 0;
	}
}

const TArray<FCirrovaTestVehicleSample>& FCirrovaTestHarness::GetSamples(const int32 Index) const
{
	static const TArray<FCirrovaTestVehicleSample> Empty;
	return Samples.IsValidIndex(Index) ? Samples[Index] : Empty;
}

const FCirrovaTestVehicleSample* FCirrovaTestHarness::GetLatest(const int32 Index) const
{
	if (!Samples.IsValidIndex(Index) || Samples[Index].IsEmpty()) { return nullptr; }
	return &Samples[Index].Last();
}

void FCirrovaTestHarness::BeginCase(const FString& InCaseName)
{
	EndCase();

	CaseName = InCaseName;
	Time = 0.0;
	PhysicsStepMs.Reset();
	Samples.SetNum(Vehicles.Num());
	for (TArray<FCirrovaTestVehicleSample>& VehicleSamples : Samples) { VehicleSamples.Reset(); }

	const FString FilePath = FPaths::Combine(OutputDir, SanitizeFileStem(CaseName) + TEXT(".csv"));
	CsvFile.Reset(IFileManager::Get().CreateFileWriter(*FilePath, EFileWrite::FILEWRITE_AllowRead));
	if (!CsvFile)
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 测试：无法创建 CSV %s。"), *FilePath);
		return;
	}

	const FString Header = TEXT(
		"time_s,case,label,vehicle,step_ms,active_constraints,registered_vehicles,"
		"x_cm,y_cm,z_cm,vx_cm_s,vy_cm_s,vz_cm_s,wx_deg_s,wy_deg_s,wz_deg_s,"
		"qx,qy,qz,qw,grounded,rpm,coupling,substeps,solver_dt_ms,"
		"wheel,contact,load_n,spring_m,compression_m,compression_vel_m_s,compression_clamped,"
		"slip_ratio,slip_angle_deg,relaxed_slip_angle_deg,fx_n,fy_n,omega_rad_s,surface_speed_m_s,"
		"effective_inertia,friction_limit_n,lateral_limit_n,util,native_active,using_constraint_length,"
		"constraint_length_m,antiroll_n,rolling_n,tire_constraint_applied,"
		"long_impulse_ns,lat_impulse_ns,impulse_window_s,constraint_wheel_inv_mass_1_kg,"
		"static_friction_limit_n,static_lateral_limit_n,kinetic_friction_limit_n,kinetic_lateral_limit_n,"
		"ground_friction,ground_static_friction,relaxed_surface_speed_m_s\r\n"
	);
	FTCHARToUTF8 Utf8(*Header);
	CsvFile->Serialize(reinterpret_cast<uint8*>(const_cast<ANSICHAR*>(Utf8.Get())), Utf8.Length());
	CsvRowsSinceFlush = 0;
}

void FCirrovaTestHarness::EndCase()
{
	if (!CsvFile) { return; }

	CsvFile->Flush();
	CsvFile.Reset();
	CsvRowsSinceFlush = 0;
}

bool FCirrovaTestHarness::EnsureCsvOpen() const { return CsvFile.IsValid(); }

void FCirrovaTestHarness::WriteReport(
	const TArray<FCirrovaTestScenarioResult>& Results,
	const bool bAllPassed
) const
{
	const FString JsonPath = FPaths::Combine(OutputDir, TEXT("report.json"));
	FString Json;
	const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
		TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);

	const TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
	Root->SetStringField(TEXT("generated"), FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")));
	Root->SetStringField(TEXT("output_dir"), OutputDir);
	Root->SetBoolField(TEXT("all_passed"), bAllPassed);

	TArray<TSharedPtr<FJsonValue>> ScenarioValues;
	for (const FCirrovaTestScenarioResult& Result : Results)
	{
		const TSharedPtr<FJsonObject> ScenarioObject = MakeShared<FJsonObject>();
		ScenarioObject->SetStringField(TEXT("name"), Result.Name);
		ScenarioObject->SetBoolField(TEXT("passed"), Result.bPassed);
		ScenarioObject->SetNumberField(TEXT("duration_s"), Result.Duration);
		ScenarioObject->SetStringField(TEXT("summary"), Result.Summary);

		TArray<TSharedPtr<FJsonValue>> FailureValues;
		for (const FString& Failure : Result.Failures)
		{
			FailureValues.Add(MakeShared<FJsonValueString>(Failure));
		}
		ScenarioObject->SetArrayField(TEXT("failures"), FailureValues);

		const TSharedPtr<FJsonObject> MetricObject = MakeShared<FJsonObject>();
		for (const FCirrovaTestMetric& Metric : Result.Metrics)
		{
			MetricObject->SetNumberField(Metric.Name, Metric.Value);
		}
		ScenarioObject->SetObjectField(TEXT("metrics"), MetricObject);
		ScenarioValues.Add(MakeShared<FJsonValueObject>(ScenarioObject));
	}
	Root->SetArrayField(TEXT("scenarios"), ScenarioValues);

	FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);

	FFileHelper::SaveStringToFile(Json, *JsonPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);

	FString Summary;
	Summary += TEXT("Cirrova P0 自动化验证\n");
	Summary += FString::Printf(TEXT("生成时间：%s\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")));
	Summary += FString::Printf(TEXT("总结果：%s\n\n"), bAllPassed ? TEXT("PASS") : TEXT("FAIL"));
	for (const FCirrovaTestScenarioResult& Result : Results)
	{
		Summary += FString::Printf(
			TEXT("%s %s %.2fs %s\n"),
			*Result.Name,
			Result.bPassed ? TEXT("PASS") : TEXT("FAIL"),
			Result.Duration,
			*Result.Summary
		);
		for (const FString& Failure : Result.Failures) { Summary += TEXT("  - ") + Failure + TEXT("\n"); }
		for (const FCirrovaTestMetric& Metric : Result.Metrics)
		{
			Summary += FString::Printf(TEXT("  %s = %.6f\n"), *Metric.Name, Metric.Value);
		}
		Summary += TEXT("\n");
	}
	Summary += TEXT("报告：") + JsonPath + TEXT("\n");
	FFileHelper::SaveStringToFile(
		Summary,
		*FPaths::Combine(OutputDir, TEXT("summary.txt")),
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM
	);

	UE_LOG(LogCirrova, Display, TEXT("Cirrova 测试：报告已写入 %s。"), *JsonPath);
}

int32 FCirrovaTestHarness::GetRegisteredVehicleCount() const
{
	return UCirrovaDiagnostics::GetRegisteredVehicleCount(World);
}

int32 FCirrovaTestHarness::GetActiveWheelConstraintCount() const
{
	return UCirrovaDiagnostics::GetActiveWheelConstraintCount(World);
}

float FCirrovaTestHarness::GetLastStepMs() const
{
	return UCirrovaDiagnostics::GetLastPhysicsStepMs(World);
}

float FCirrovaTestHarness::GetAverageStepMs() const
{
	return UCirrovaDiagnostics::GetAveragePhysicsStepMs(World);
}

float FCirrovaTestHarness::GetPeakStepMs() const
{
	return UCirrovaDiagnostics::GetPeakPhysicsStepMs(World);
}

FVector FCirrovaTestHarness::GetTestOrigin()
{
	// 远离默认关卡出生点，避免玩家 Pawn 与测试地面/载具相互干扰。
	return FVector(100000.0f, 100000.0f, 0.0f);
}

FTransform FCirrovaTestHarness::MakeFlatGround(const FVector& Center, const float ExtentM)
{
	const float Scale = FMath::Max(ExtentM, 1.0f) * 2.0f;
	return FTransform(
		FQuat::Identity,
		Center + FVector(0.0f, 0.0f, -50.0f),
		FVector(Scale, Scale, 1.0f)
	);
}

FTransform FCirrovaTestHarness::MakeSlopedGround(
	const FVector& Center,
	const float RollDegrees,
	const float PitchDegrees
)
{
	const FQuat Rotation(FRotator(PitchDegrees, 0.0f, RollDegrees));
	const FVector Normal = Rotation.GetAxisZ();
	return FTransform(
		Rotation,
		Center - Normal * 50.0f,
		FVector(200.0f, 200.0f, 1.0f)
	);
}

FTransform FCirrovaTestHarness::MakeBox(
	const FVector& Center,
	const FVector& FullSizeCm,
	const float YawDegrees
)
{
	const FQuat Rotation(FRotator(0.0f, YawDegrees, 0.0f));
	const FVector Scale = FVector(
		FMath::Max(FullSizeCm.X, 1.0f) / 100.0f,
		FMath::Max(FullSizeCm.Y, 1.0f) / 100.0f,
		FMath::Max(FullSizeCm.Z, 1.0f) / 100.0f
	);
	return FTransform(Rotation, Center, Scale);
}

FTransform FCirrovaTestHarness::MakeVehicleOnGround(
	const FTransform& Ground,
	const float WheelBottomHeightM,
	const float ForwardOffsetM,
	const float LateralOffsetM,
	const float YawOffsetDegrees
)
{
	const FVector TopCenter = Ground.TransformPosition(FVector(0.0f, 0.0f, 50.0f));
	const FQuat Rotation = Ground.GetRotation();
	const FVector Normal = Rotation.GetAxisZ();
	const FVector Forward = Rotation.GetAxisX();
	const FVector Right = Rotation.GetAxisY();

	const FVector Location = TopCenter
		+ Normal * ((SuspensionRestLengthM + WheelRadiusM + WheelBottomHeightM) * 100.0f)
		+ Forward * (ForwardOffsetM * 100.0f)
		+ Right * (LateralOffsetM * 100.0f);
	const FQuat VehicleRotation = Rotation * FQuat(FVector::UpVector, FMath::DegreesToRadians(YawOffsetDegrees));
	return FTransform(VehicleRotation, Location);
}
