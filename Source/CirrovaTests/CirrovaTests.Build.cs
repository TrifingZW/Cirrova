using UnrealBuildTool;

public class CirrovaTests : ModuleRules
{
	public CirrovaTests(ReadOnlyTargetRules target) : base(target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			[
				"Core",
				"CoreUObject",
				"Engine",
				"Cirrova"
			]
		);

		PrivateDependencyModuleNames.AddRange(
			[
				"Json"
			]
		);
	}
}
