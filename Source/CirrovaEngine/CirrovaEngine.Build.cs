using UnrealBuildTool;

public class CirrovaEngine : ModuleRules
{
	public CirrovaEngine(ReadOnlyTargetRules target) : base(target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(
			[
				"Core",
				"CoreUObject",
				"Engine"
			]
		);
	}
}