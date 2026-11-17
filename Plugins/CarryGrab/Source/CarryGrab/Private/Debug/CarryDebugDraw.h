#pragma once

#include "Containers/Array.h"
#include "DrawDebugHelpers.h"
#include "Math/Vector.h"

class AActor;
class UPrimitiveComponent;
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
    void DrawGripMarkers(const UWorld& World, const UPrimitiveComponent& Body, const FCarryGrip& Grip,
        int32 GripIndex, const FVector& Contact, const FVector& Normal, int32 GripCount);
    void DrawItemDebug(const UWorld& World, const AActor& Owner, const UPrimitiveComponent& Body,
        const TArray<FCarryGrip>& Grips, int32 EffectiveMaxHolders);
}
