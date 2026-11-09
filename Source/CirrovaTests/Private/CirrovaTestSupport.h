#pragma once

#include "CoreMinimal.h"
#include "CirrovaTestHarness.h"
#include "CirrovaTestVehicle.h"

namespace CirrovaTestSupport
{
	constexpr float Gravity = 9.81f;

	/** 单轮在时间窗内的统计量。 */
	struct FWheelStat
	{
		int32 Count = 0;
		float ContactFraction = 0.0f;
		float NativeFraction = 0.0f;
		float ConstraintFraction = 0.0f;
		float AppliedFraction = 0.0f;
		float ClampFraction = 0.0f;
		float MeanLoad = 0.0f;
		float MeanSpring = 0.0f;
		float MeanCompression = 0.0f;
		float MeanAbsCompressionVelocity = 0.0f;
		float MeanAbsSlip = 0.0f;
		float MeanAbsAlpha = 0.0f;
		float MeanAbsRelaxedAlpha = 0.0f;
		float MeanFrictionLimit = 0.0f;
		float MaxFrictionLimit = 0.0f;
		float MeanUtilization = 0.0f;
		float MaxUtilization = 0.0f;
		float MeanAbsLateralForce = 0.0f;
		float MeanLongitudinalForce = 0.0f;
		float MeanAbsOmega = 0.0f;
		float MaxAbsLateralForce = 0.0f;
		float MaxAbsLongitudinalForce = 0.0f;
	};

	/** 整车在时间窗内的统计量。 */
	struct FVehicleStat
	{
		int32 Count = 0;
		bool bAllFinite = true;
		FVector StartLocation = FVector::ZeroVector;
		FVector EndLocation = FVector::ZeroVector;
		float MeanSpeed = 0.0f;
		float MaxSpeed = 0.0f;
		float MaxAbsVerticalSpeed = 0.0f;
		float MaxTiltDegrees = 0.0f;
		float MinUpDot = 1.0f;
		float MaxAbsPitch = 0.0f;
		float MaxAbsRoll = 0.0f;
		float MeanSumLoad = 0.0f;
	};

	inline bool IsSampleInWindow(
		const FCirrovaTestVehicleSample& Sample,
		const double Start,
		const double End
	)
	{
		return Sample.Time >= Start && Sample.Time <= End;
	}

	inline FWheelStat ComputeWheelStat(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const int32 WheelIndex,
		const double Start,
		const double End
	)
	{
		FWheelStat Stat;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (!IsSampleInWindow(Sample, Start, End) || !Sample.Wheels.IsValidIndex(WheelIndex)) { continue; }

			const FCirrovaWheelState& Wheel = Sample.Wheels[WheelIndex];
			++Stat.Count;
			Stat.ContactFraction += Wheel.bInContact ? 1.0f : 0.0f;
			Stat.NativeFraction += Wheel.bNativeSuspensionActive ? 1.0f : 0.0f;
			Stat.ConstraintFraction += Wheel.bUsingConstraintLength ? 1.0f : 0.0f;
			Stat.AppliedFraction += Wheel.bTireConstraintApplied ? 1.0f : 0.0f;
			Stat.ClampFraction += Wheel.bCompressionVelocityClamped ? 1.0f : 0.0f;
			Stat.MeanLoad += Wheel.NormalLoad;
			Stat.MeanSpring += Wheel.SuspensionLength;
			Stat.MeanCompression += Wheel.Compression;
			Stat.MeanAbsCompressionVelocity += FMath::Abs(Wheel.CompressionVelocity);
			Stat.MeanAbsSlip += FMath::Abs(Wheel.SlipRatio);
			Stat.MeanAbsAlpha += FMath::Abs(Wheel.SlipAngleDegrees);
			Stat.MeanAbsRelaxedAlpha += FMath::Abs(Wheel.RelaxedSlipAngleDegrees);
			Stat.MeanFrictionLimit += Wheel.FrictionForceLimit;
			Stat.MaxFrictionLimit = FMath::Max(Stat.MaxFrictionLimit, Wheel.FrictionForceLimit);
			Stat.MeanUtilization += Wheel.FrictionUtilization;
			Stat.MaxUtilization = FMath::Max(Stat.MaxUtilization, Wheel.FrictionUtilization);
			Stat.MeanAbsLateralForce += FMath::Abs(Wheel.TireLateralForce);
			Stat.MeanLongitudinalForce += Wheel.TireLongitudinalForce;
			Stat.MeanAbsOmega += FMath::Abs(Wheel.Omega);
			Stat.MaxAbsLateralForce = FMath::Max(Stat.MaxAbsLateralForce, FMath::Abs(Wheel.TireLateralForce));
			Stat.MaxAbsLongitudinalForce =
				FMath::Max(Stat.MaxAbsLongitudinalForce, FMath::Abs(Wheel.TireLongitudinalForce));
		}

		if (Stat.Count <= 0) { return Stat; }

		const float InvCount = 1.0f / static_cast<float>(Stat.Count);
		Stat.ContactFraction *= InvCount;
		Stat.NativeFraction *= InvCount;
		Stat.ConstraintFraction *= InvCount;
		Stat.AppliedFraction *= InvCount;
		Stat.ClampFraction *= InvCount;
		Stat.MeanLoad *= InvCount;
		Stat.MeanSpring *= InvCount;
		Stat.MeanCompression *= InvCount;
		Stat.MeanAbsCompressionVelocity *= InvCount;
		Stat.MeanAbsSlip *= InvCount;
		Stat.MeanAbsAlpha *= InvCount;
		Stat.MeanAbsRelaxedAlpha *= InvCount;
		Stat.MeanFrictionLimit *= InvCount;
		Stat.MeanUtilization *= InvCount;
		Stat.MeanAbsLateralForce *= InvCount;
		Stat.MeanLongitudinalForce *= InvCount;
		Stat.MeanAbsOmega *= InvCount;
		return Stat;
	}

	inline FVehicleStat ComputeVehicleStat(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const double Start,
		const double End,
		const FVector& PlaneNormal
	)
	{
		FVehicleStat Stat;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (!IsSampleInWindow(Sample, Start, End)) { continue; }

			if (Stat.Count == 0) { Stat.StartLocation = Sample.Location; }
			Stat.EndLocation = Sample.Location;
			++Stat.Count;
			Stat.bAllFinite &= Sample.IsFinite();

			const float Speed = Sample.Velocity.Size() * 0.01f;
			Stat.MeanSpeed += Speed;
			Stat.MaxSpeed = FMath::Max(Stat.MaxSpeed, Speed);
			Stat.MaxAbsVerticalSpeed = FMath::Max(Stat.MaxAbsVerticalSpeed, FMath::Abs(Sample.Velocity.Z) * 0.01f);

			const FVector Up = Sample.Rotation.GetAxisZ();
			const float UpDot = FMath::Clamp(FVector::DotProduct(Up, PlaneNormal), -1.0f, 1.0f);
			Stat.MinUpDot = FMath::Min(Stat.MinUpDot, UpDot);
			Stat.MaxTiltDegrees = FMath::Max(
				Stat.MaxTiltDegrees,
				FMath::RadiansToDegrees(FMath::Acos(UpDot))
			);

			const FRotator Rotator = Sample.Rotation.Rotator();
			Stat.MaxAbsPitch = FMath::Max(Stat.MaxAbsPitch, FMath::Abs(Rotator.Pitch));
			Stat.MaxAbsRoll = FMath::Max(Stat.MaxAbsRoll, FMath::Abs(Rotator.Roll));

			float SumLoad = 0.0f;
			for (const FCirrovaWheelState& Wheel : Sample.Wheels) { SumLoad += Wheel.NormalLoad; }
			Stat.MeanSumLoad += SumLoad;
		}

		if (Stat.Count <= 0) { return Stat; }
		const float InvCount = 1.0f / static_cast<float>(Stat.Count);
		Stat.MeanSpeed *= InvCount;
		Stat.MeanSumLoad *= InvCount;
		return Stat;
	}

	inline float MeanTotalLoad(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const double Start,
		const double End
	)
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (!IsSampleInWindow(Sample, Start, End)) { continue; }
			++Count;
			for (const FCirrovaWheelState& Wheel : Sample.Wheels) { Sum += Wheel.NormalLoad; }
		}
		return Count > 0 ? Sum / static_cast<float>(Count) : 0.0f;
	}

	inline float MeanAbsLateralForce(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const TArray<int32>& WheelIndices,
		const double Start,
		const double End
	)
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (!IsSampleInWindow(Sample, Start, End)) { continue; }
			for (const int32 WheelIndex : WheelIndices)
			{
				if (!Sample.Wheels.IsValidIndex(WheelIndex)) { continue; }
				Sum += FMath::Abs(Sample.Wheels[WheelIndex].TireLateralForce);
				++Count;
			}
		}
		return Count > 0 ? Sum / static_cast<float>(Count) : 0.0f;
	}

	inline int32 CountSignChanges(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const int32 WheelIndex,
		const double Start,
		const double End,
		const float Threshold
	)
	{
		int32 Changes = 0;
		int32 LastSign = 0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (!IsSampleInWindow(Sample, Start, End) || !Sample.Wheels.IsValidIndex(WheelIndex)) { continue; }

			const float Value = Sample.Wheels[WheelIndex].TireLongitudinalForce;
			if (FMath::Abs(Value) < Threshold) { continue; }
			const int32 Sign = Value > 0.0f ? 1 : -1;
			if (LastSign != 0 && Sign != LastSign) { ++Changes; }
			LastSign = Sign;
		}
		return Changes;
	}

	inline float FindSettledTime(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const double SearchStart,
		const float MaxCompressionVelocity,
		const float StableSeconds
	)
	{
		double StableStart = -1.0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (Sample.Time < SearchStart) { continue; }

			bool bStable = true;
			for (const FCirrovaWheelState& Wheel : Sample.Wheels)
			{
				if (!Wheel.bInContact || FMath::Abs(Wheel.CompressionVelocity) > MaxCompressionVelocity)
				{
					bStable = false;
					break;
				}
			}

			if (!bStable)
			{
				StableStart = -1.0;
				continue;
			}

			if (StableStart < 0.0) { StableStart = Sample.Time; }
			if (Sample.Time - StableStart >= StableSeconds) { return static_cast<float>(StableStart); }
		}
		return -1.0f;
	}

	inline float FirstContactTime(const TArray<FCirrovaTestVehicleSample>& Samples)
	{
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			for (const FCirrovaWheelState& Wheel : Sample.Wheels)
			{
				if (Wheel.bInContact) { return static_cast<float>(Sample.Time); }
			}
		}
		return -1.0f;
	}

	inline float MeanWheelOmega(
		const TArray<FCirrovaTestVehicleSample>& Samples,
		const double Start,
		const double End
	)
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (const FCirrovaTestVehicleSample& Sample : Samples)
		{
			if (!IsSampleInWindow(Sample, Start, End)) { continue; }
			for (const FCirrovaWheelState& Wheel : Sample.Wheels)
			{
				Sum += FMath::Abs(Wheel.Omega);
				++Count;
			}
		}
		return Count > 0 ? Sum / static_cast<float>(Count) : 0.0f;
	}

	inline FString WheelMetricName(const int32 WheelIndex, const TCHAR* Name)
	{
		return FString::Printf(TEXT("w%d_%s"), WheelIndex, Name);
	}

	inline FString VehicleMetricName(const int32 VehicleIndex, const TCHAR* Name)
	{
		return FString::Printf(TEXT("v%d_%s"), VehicleIndex, Name);
	}

	inline float GetSpeedKmh(const FCirrovaTestVehicleSample* Sample)
	{
		return Sample ? Sample->Velocity.Size() * 0.01f * 3.6f : 0.0f;
	}

	inline void SetHeldVelocity(
		ACirrovaTestVehicle* Vehicle,
		const FVector& Direction,
		const float SpeedMps
	)
	{
		if (!Vehicle) { return; }
		Vehicle->SetHorizontalVelocity(Direction.GetSafeNormal() * SpeedMps * 100.0f);
	}

	inline void ComputeChassisAabb(
		const FCirrovaTestVehicleSample& Sample,
		const FVector& LocalExtent,
		FVector& OutMin,
		FVector& OutMax
	)
	{
		OutMin = FVector(MAX_flt, MAX_flt, MAX_flt);
		OutMax = FVector(-MAX_flt, -MAX_flt, -MAX_flt);
		for (int32 SignX = -1; SignX <= 1; SignX += 2)
		{
			for (int32 SignY = -1; SignY <= 1; SignY += 2)
			{
				for (int32 SignZ = -1; SignZ <= 1; SignZ += 2)
				{
					const FVector Corner = Sample.Location + Sample.Rotation.RotateVector(
						FVector(SignX * LocalExtent.X, SignY * LocalExtent.Y, SignZ * LocalExtent.Z)
					);
					OutMin = OutMin.ComponentMin(Corner);
					OutMax = OutMax.ComponentMax(Corner);
				}
			}
		}
	}
}
