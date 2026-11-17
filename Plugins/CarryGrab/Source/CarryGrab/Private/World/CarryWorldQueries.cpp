#include "World/CarryWorldQueries.h"

#include "CarryableComponent.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"
#include "Components/CapsuleComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "PlayerCarryComponent.h"

namespace CarryWorld
{
    bool TracePickup(UWorld& World, const APawn& Pawn, const FTransform& View, float ReachDistance, FHitResult& Hit)
    {
        const FVector Start = View.GetLocation();
        const FVector End = View.TransformPosition(FVector(ReachDistance, 0.0, 0.0));
        const FCollisionQueryParams Query(SCENE_QUERY_STAT(CarryGrab), false, &Pawn);
        return World.LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Query) && Hit.GetActor();
    }

    FGripReach EvaluateGripReach(UWorld& World, const APawn& Holder, const UPrimitiveComponent& Body,
        const FVector& Contact, const FVector& Normal)
    {
        static const FCarryPlayerSettings Defaults;
        const UPlayerCarryComponent* PlayerCarry = Holder.FindComponentByClass<UPlayerCarryComponent>();
        const FCarryPlayerSettings* Profile = PlayerCarry ? PlayerCarry->GetProfileSettings() : nullptr;
        const FCarryPlayerSettings& Settings = Profile ? *Profile : Defaults;
        const UPlayerCarryComponent* ProbeSettings = PlayerCarry ? PlayerCarry : GetDefault<UPlayerCarryComponent>();

        const UCapsuleComponent* Capsule = Cast<UCapsuleComponent>(Holder.GetRootComponent());
        const FVector Center = Capsule ? Capsule->GetComponentLocation() : Holder.GetActorLocation();
        FGripReach Reach;
        Reach.Start = Center + FVector::UpVector * Settings.ElbowHeightOffset;
        const FVector ToContact = Contact - Reach.Start;
        const double Distance = ToContact.Size();
        if (Distance > FMath::Max(1.0f, Settings.HandReachDistance))
        {
            return Reach;
        }

        Reach.Radius = FMath::Max(1.0f, ProbeSettings->GetHandSweepRadius());
        constexpr double SurfaceClearanceCm = 0.25;
        constexpr double MaxFirstSweepFraction = 0.75;
        constexpr double UndersideNormalZ = -0.6;
        constexpr double HitPositionPaddingCm = 1.0;

        const double Clearance = Reach.Radius + SurfaceClearanceCm;
        const FVector Forward = ToContact.GetSafeNormal2D(
            UE_SMALL_NUMBER, Holder.GetActorForwardVector().GetSafeNormal2D());
        const double Rise = FMath::DegreesToRadians(FMath::Clamp<double>(Settings.ProbeRiseDegrees, 0.0, 89.0));
        const FVector FirstDirection = Forward * FMath::Cos(Rise) + FVector::UpVector * FMath::Sin(Rise);
        double FirstLength = FMath::Min<double>(FMath::Max(1.0f, ProbeSettings->GetProbeFirstLength()), MaxFirstSweepFraction * Distance);

        const double OriginGap = FVector::DotProduct(Reach.Start - Contact, Normal);
        const double GapChange = FVector::DotProduct(FirstDirection, Normal);
        if (Normal.Z <= UndersideNormalZ && OriginGap > Clearance && GapChange < -UE_SMALL_NUMBER)
        {
            // Keep the bend below an underside surface when approaching it from below.
            FirstLength = FMath::Min(FirstLength, (OriginGap - Clearance) / -GapChange);
        }

        Reach.Bend = Reach.Start + FirstDirection * FirstLength;
        // Stop the sphere center outside the face so its surface approaches the intended contact.
        Reach.End = Contact + Normal * Clearance;
        const FCollisionQueryParams Query(SCENE_QUERY_STAT(CarryBentReach), false, &Holder);
        const FCollisionShape Shape = FCollisionShape::MakeSphere(Reach.Radius);
        FHitResult Hit;
        if (World.SweepSingleByChannel(Hit, Reach.Start, Reach.Bend, FQuat::Identity, ECC_Visibility, Shape, Query))
        {
            Reach.End = Hit.Location;
            Reach.Block = EReachBlock::FirstSweep;
            return Reach;
        }

        if (World.SweepSingleByChannel(Hit, Reach.Bend, Reach.End, FQuat::Identity, ECC_Visibility, Shape, Query))
        {
            Reach.End = Hit.Location;
            const bool bReachedGrip = !Hit.bStartPenetrating && Hit.GetComponent() == &Body
                && FVector::DistSquared(Hit.ImpactPoint, Contact) <= FMath::Square(2.0 * Reach.Radius + HitPositionPaddingCm);
            Reach.Block = bReachedGrip ? EReachBlock::None : EReachBlock::SecondSweep;
        }
        return Reach;
    }

    bool FindRegripContact(UWorld& World, const APawn& Holder, const UPrimitiveComponent& Body, const FTransform& View,
        const FVector& ReachStart, const FVector& Contact, const FCarryPlayerSettings& Settings, double MinGripChange,
        FHitResult& Hit)
    {
        const FVector Eye = View.GetLocation();
        const FCollisionQueryParams Query(SCENE_QUERY_STAT(CarryAutoRegrip), false, &Holder);
        const auto FindNewContact = [&](const FVector& End)
        {
            return World.LineTraceSingleByChannel(Hit, Eye, End, ECC_Visibility, Query)
                && Hit.GetComponent() == &Body
                && !Hit.bStartPenetrating
                && FVector::DistSquared(Eye, Hit.ImpactPoint) <= FMath::Square(FMath::Max(1.0f, Settings.PickupReach))
                && FVector::DistSquared(ReachStart, Hit.ImpactPoint) <= FMath::Square(FMath::Max(1.0f, Settings.HandReachDistance))
                && FVector::DistSquared(Contact, Hit.ImpactPoint) >= FMath::Square(MinGripChange);
        };
        return FindNewContact(View.TransformPosition(FVector(Settings.PickupReach, 0.0, 0.0)))
            || FindNewContact(Body.GetCenterOfMass());
    }

    FHeavyApproachHit SweepHeavyApproach(UWorld& World, const AActor* ItemOwner, UPrimitiveComponent& Body,
        const TArray<FCarryGrip>& Grips, const FVector& Direction, double Distance, double Mass, double MinMassRatio)
    {
        FComponentQueryParams Query(SCENE_QUERY_STAT(CarryHeavyApproach), ItemOwner);
        Query.bFindInitialOverlaps = false;
        for (const FCarryGrip& Grip : Grips)
        {
            Query.AddIgnoredActor(Grip.Holder.Get());
        }
        const FVector Start = Body.GetComponentLocation();
        TArray<FHitResult> Hits;
        World.ComponentSweepMulti(Hits, &Body, Start, Start + Direction * Distance, Body.GetComponentQuat(), Query);
        for (const FHitResult& Hit : Hits)
        {
            UPrimitiveComponent* Other = Hit.GetComponent();
            const FVector Normal = Hit.ImpactNormal.GetSafeNormal2D();
            if (!Hit.bBlockingHit || Hit.bStartPenetrating || FMath::Abs(Hit.ImpactNormal.Z) > 0.5
                || FVector::DotProduct(Normal, Direction) >= -0.2 || !IsValid(Other) || !Other->IsSimulatingPhysics(Hit.BoneName))
            {
                continue;
            }

            const double OtherMass = Other->GetMass();
            if (OtherMass <= Mass * MinMassRatio || !Other->GetOwner()->FindComponentByClass<UCarryableComponent>())
            {
                continue;
            }
            return { Other, Normal, Hit.Location - Other->GetComponentLocation(), OtherMass };
        }
        return {};
    }

    bool OverlapsHolder(UPrimitiveComponent& ItemBody, UPrimitiveComponent& HolderBody)
    {
        return ItemBody.ComponentOverlapComponent(&HolderBody, HolderBody.GetComponentLocation(),
            HolderBody.GetComponentQuat(), FCollisionQueryParams::DefaultQueryParam);
    }
}
