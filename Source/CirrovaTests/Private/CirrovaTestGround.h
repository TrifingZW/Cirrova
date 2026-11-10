#pragma once

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "GameFramework/Actor.h"
#include "CirrovaTestGround.generated.h"

class UBoxComponent;
class UPhysicalMaterial;

/**
 * 自动化测试用静态地面/台阶。
 *
 * 根组件为静态盒体，尺寸由 Actor 缩放决定：默认盒体全尺寸 100 cm，
 * 缩放 (200, 200, 1) 即 200 m × 200 m × 1 m 的地面。碰撞体为简单盒形，
 * 在 Chaos 中生成静态单粒子代理，可被原生悬挂约束用作目标体。
 * 地面材质可在生成前配置，用于验证材质摩擦查询。
 */
UCLASS()
class ACirrovaTestGround : public AActor
{
	GENERATED_BODY()

public:
	ACirrovaTestGround();

	// 静态碰撞盒。
	UPrimitiveComponent* GetCollision() const { return Collision.Get(); }

	// 在 FinishSpawning 前写入地面动/静摩擦系数。
	void ConfigureSurface(float Friction, float StaticFriction);

	virtual void PostInitializeComponents() override;

private:
	void ApplySurfaceMaterial();

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UBoxComponent> Collision;

	UPROPERTY()
	TObjectPtr<UPhysicalMaterial> SurfaceMaterial;

	float SurfaceFriction = 1.0f;
	float SurfaceStaticFriction = 1.0f;
};
