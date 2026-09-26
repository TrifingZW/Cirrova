using UnrealBuildTool;

public class CirrovaControl : ModuleRules
{
	public CirrovaControl(ReadOnlyTargetRules target) : base(target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(
			[
				"Core",
				"CoreUObject",
				"Engine",

				"InputCore",
				"EnhancedInput",

				"GameplayTags"
			]
		);
	}
}