#include "CirrovaTestGround.h"

#include "Components/BoxComponent.h"

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
