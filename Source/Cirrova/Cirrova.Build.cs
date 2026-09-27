using UnrealBuildTool;

public class Cirrova : ModuleRules
{
	public Cirrova(ReadOnlyTargetRules target) : base(target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(
			[
				"Core",
				"CoreUObject",
				"Engine",

				"ChaosVehiclesCore",

				"InputCore",
				"EnhancedInput",

				"GameplayTags",

				"SingularisMorphVehicle"
			]
		);
	}
}