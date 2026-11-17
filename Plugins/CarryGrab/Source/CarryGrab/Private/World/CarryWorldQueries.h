#pragma once

#include "CoreMinimal.h"

class AActor;
class APawn;
class UPrimitiveComponent;
class UWorld;
struct FCarryGrip;
struct FCarryPlayerSettings;
struct FHitResult;

namespace CarryWorld
{
    enum class EReachBlock : uint8
    {
        None,
        FirstSweep,
        SecondSweep
    };

    struct FGripReach
    {
        FVector Start = FVector::ZeroVector;
        FVector Bend = FVector::ZeroVector;
        FVector End = FVector::ZeroVector;
        float Radius = 0.0f;
        EReachBlock Block = EReachBlock::None;

        bool CanReach() const { return Radius > 0.0f && Block == EReachBlock::None; }
        bool IsBlocked() const { return Block != EReachBlock::None; }
    };

    struct FHeavyApproachHit
    {
        UPrimitiveComponent* Body = nullptr;
        FVector Normal = FVector::ZeroVector;
        FVector StopOffset = FVector::ZeroVector;
        double Mass = 0.0;
    };

    bool TracePickup(UWorld& World, const APawn& Pawn, const FTransform& View, float ReachDistance, FHitResult& Hit);
    FGripReach EvaluateGripReach(UWorld& World, const APawn& Holder, const UPrimitiveComponent& Body,
        const FVector& Contact, const FVector& Normal);
    bool FindRegripContact(UWorld& World, const APawn& Holder, const UPrimitiveComponent& Body, const FTransform& View,
        const FVector& ReachStart, const FVector& Contact, const FCarryPlayerSettings& Settings, double MinGripChange,
        FHitResult& Hit);
    FHeavyApproachHit SweepHeavyApproach(UWorld& World, const AActor* ItemOwner, UPrimitiveComponent& Body,
        const TArray<FCarryGrip>& Grips, const FVector& Direction, double Distance, double Mass, double MinApproachMassRatio);
    bool HasNearbySupport(UWorld& World, UPrimitiveComponent& Body, const APawn& Holder, const FTransform& BodyPose);
    bool OverlapsHolder(UPrimitiveComponent& ItemBody, UPrimitiveComponent& HolderBody);
}
