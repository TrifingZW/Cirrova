#include "CirrovaDiagnostics.h"

#include "CirrovaSchedulerSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

namespace
{
	/** 从世界上下文解析调度器；世界或子系统不可用时返回空。 */
	const UCirrovaSchedulerSubsystem* ResolveScheduler(const UObject* WorldContextObject)
	{
		const UWorld* World = GEngine
			? GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull)
			: nullptr;
		return World ? World->GetSubsystem<UCirrovaSchedulerSubsystem>() : nullptr;
	}
}

int32 UCirrovaDiagnostics::GetRegisteredVehicleCount(const UObject* WorldContextObject)
{
	const UCirrovaSchedulerSubsystem* Scheduler = ResolveScheduler(WorldContextObject);
	return Scheduler ? Scheduler->GetRegisteredVehicleCount() : 0;
}

int32 UCirrovaDiagnostics::GetActiveWheelConstraintCount(const UObject* WorldContextObject)
{
	const UCirrovaSchedulerSubsystem* Scheduler = ResolveScheduler(WorldContextObject);
	return Scheduler ? Scheduler->GetActiveWheelConstraintCount() : 0;
}

float UCirrovaDiagnostics::GetLastPhysicsStepMs(const UObject* WorldContextObject)
{
	const UCirrovaSchedulerSubsystem* Scheduler = ResolveScheduler(WorldContextObject);
	return Scheduler ? Scheduler->GetLastPhysicsStepMs() : 0.0f;
}

float UCirrovaDiagnostics::GetAveragePhysicsStepMs(const UObject* WorldContextObject)
{
	const UCirrovaSchedulerSubsystem* Scheduler = ResolveScheduler(WorldContextObject);
	return Scheduler ? Scheduler->GetAveragePhysicsStepMs() : 0.0f;
}

float UCirrovaDiagnostics::GetPeakPhysicsStepMs(const UObject* WorldContextObject)
{
	const UCirrovaSchedulerSubsystem* Scheduler = ResolveScheduler(WorldContextObject);
	return Scheduler ? Scheduler->GetPeakPhysicsStepMs() : 0.0f;
}
