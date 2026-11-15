using UnrealBuildTool;

public class CarryGrab : ModuleRules
{
    public CarryGrab(ReadOnlyTargetRules Target) : base(Target)
    {
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new[] { "PhysicsCore", "Chaos" });
    }
}
