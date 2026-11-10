#include "CirrovaTestGround.h"

#include "Components/BoxComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

ACirrovaTestGround::ACirrovaTestGround()
{
	PrimaryActorTick.bCanEverTick = false;

	Collision = CreateDefaultSubobject<UBoxComponent>(TEXT("Collision"));
	SetRootComponent(Collision);
	Collision->SetMobility(EComponentMobility::Static);
	Collision->SetBoxExtent(FVector(50.0f, 50.0f, 50.0f));
	Collision->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Collision->SetCollisionObjectType(ECC_WorldStatic);
	Collision->SetCollisionResponseToAllChannels(ECR_Block);
	Collision->SetGenerateOverlapEvents(false);
	Collision->SetCanEverAffectNavigation(false);
}

void ACirrovaTestGround::ConfigureSurface(const float Friction, const float StaticFriction)
{
	SurfaceFriction = FMath::IsFinite(Friction) ? FMath::Max(Friction, 0.0f) : 1.0f;
	SurfaceStaticFriction = FMath::IsFinite(StaticFriction)
		? FMath::Max(StaticFriction, SurfaceFriction)
		: SurfaceFriction;
}


void ACirrovaTestGround::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	ApplySurfaceMaterial();
}

void ACirrovaTestGround::ApplySurfaceMaterial()
{
	if (!Collision) { return; }

	if (!SurfaceMaterial)
	{
		SurfaceMaterial = NewObject<UPhysicalMaterial>(this, TEXT("CirrovaTestGroundMaterial"));
	}
	SurfaceMaterial->Friction = SurfaceFriction;
	SurfaceMaterial->StaticFriction = SurfaceStaticFriction;
	Collision->SetPhysMaterialOverride(SurfaceMaterial);
}
