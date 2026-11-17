#pragma once

#include "DrawDebugHelpers.h"
#include "Math/Vector.h"

class UWorld;
struct FCarryGrip;

namespace CarryWorld
{
    struct FGripReach;
}

namespace CarryDebug
{
#if ENABLE_DRAW_DEBUG
    void DrawBentReachPath(const UWorld& World, const CarryWorld::FGripReach& Reach, const FVector& Contact);
#endif
    void DrawGripMarkers(const UWorld& World, const FCarryGrip& Grip, const FVector& Contact, const FVector& Normal);
}
