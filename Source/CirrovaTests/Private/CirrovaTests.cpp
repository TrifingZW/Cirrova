#include "CirrovaTests.h"

#include "CirrovaLog.h"

void FCirrovaTestsModule::StartupModule()
{
	UE_LOG(LogCirrova, Log, TEXT("Cirrova 测试模块：自动化 P0 测试已加载。"));
}

void FCirrovaTestsModule::ShutdownModule() {}

IMPLEMENT_MODULE(FCirrovaTestsModule, CirrovaTests)
