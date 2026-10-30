using System.IO;
using UnrealBuildTool;

public class Cirrova : ModuleRules
{
	public Cirrova(ReadOnlyTargetRules target) : base(target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// CirrovaEngine 持有 liminal FFI 绑定头，
		// 载具组件的公开头文件包含了 CirrovaLiminalApi.h，因此须为公共依赖。
		PublicDependencyModuleNames.Add("CirrovaEngine");

		// liminal Rust 静态库由本模块（FFI 的实际调用方）链接，路径须归一化为全限定路径。
		// 构建前须先执行：cd ThirdParty/liminal && cargo build --release
		string LiminalLibrary = Path.GetFullPath(Path.Combine(
			ModuleDirectory, "..", "..", "..", "..", "ThirdParty", "liminal", "target", "release", "liminal.lib"));
		if (!File.Exists(LiminalLibrary))
		{
			throw new BuildException(
				"未找到 liminal 静态库：{0}。请先执行 cd ThirdParty/liminal && cargo build --release",
				LiminalLibrary);
		}
		PublicAdditionalLibraries.Add(LiminalLibrary);

		// liminal 静态库内含 Rust std 的 Windows 实现，其文件/进程/环境代码引用了
		// ntdll 与 userenv 的导入符号；UE 默认链接集不含这两个库，须显式补充。
		PublicSystemLibraries.AddRange(
			[
				"ntdll.lib",
				"userenv.lib"
			]
		);

		PrivateDependencyModuleNames.AddRange(
			[
				"Core",
				"CoreUObject",
				"Engine",

				"ChaosVehiclesCore",

				"InputCore",
				"EnhancedInput",

				"GameplayTags"
			]
		);
	}
}