using UnrealBuildTool;

public class Cirrova : ModuleRules
{
	public Cirrova(ReadOnlyTargetRules target) : base(target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			[
				"SingularisMorphVehicle"
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