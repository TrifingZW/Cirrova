#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/** Cirrova 自动化 P0 测试模块；仅开发者构建加载。 */
class FCirrovaTestsModule final : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
