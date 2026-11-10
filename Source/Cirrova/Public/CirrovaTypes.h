#pragma once

#include "CoreMinimal.h"
#include "CirrovaTypes.generated.h"

/** 车轮接触检测探针形态。 */
UENUM(BlueprintType)
enum class ECirrovaWheelProbeMode : uint8
{
	// 零厚度射线，最便宜，边缘与斜坡精度最低。
	Ray UMETA(DisplayName = "射线"),
	// 球体扫掠，半径等于车轮半径，兼顾台阶与成本。
	Sphere UMETA(DisplayName = "球体扫掠"),
	// 圆柱包络：沿轮宽多射线，最接近真实轮胎包络。
	Cylinder UMETA(DisplayName = "圆柱包络")
};
