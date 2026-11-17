#include "CarryPhysicsComponent.h"

#include "Carry/CarryConstants.h"
#include "CarryableComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/BodyInstance.h"
#include "ProfilingDebugging/CsvProfiler.h"
#include "World/CarryWorldQueries.h"

CSV_DEFINE_CATEGORY(CarryContacts, false);

namespace
{
    constexpr double ContactExpirySeconds = 0.09;
    constexpr double MinMassRatio = 2.0;
    constexpr double FullLimitMassRatio = 4.0;
    constexpr double HeavyContactAcceleration = 4.0; // m/s² of additional lateral spring force into contact
    constexpr int32 MaxHeavyContacts = 4;
    constexpr double ApproachQueryInterval = 0.05;
    constexpr double ApproachExpiry = 0.12;
    constexpr double ApproachDistance = 90.0;
    constexpr double ApproachDriveThreshold = 16.0;
    constexpr double ApproachErrorFraction = 0.15;

    TAutoConsoleVariable<int32> CVarLimitHeavyContactPush(
        TEXT("carry.LimitHeavyContactPush"), 1,
        TEXT("Limit a held prop's sideways drive into much heavier movable carryables. 0: off, 1: on."));
    TAutoConsoleVariable<int32> CVarBrakeHeavyApproach(
        TEXT("carry.BrakeHeavyApproach"), 1,
        TEXT("Brake fast held props approaching a much heavier movable carryable. 0: off, 1: on."));
}

void UCarryPhysicsComponent::RefreshHeavyObstacleTracking(const AActor& Owner, UPrimitiveComponent* Body, double Mass)
{
    TrackedBodyMass = Mass;
    SetContactTracking(CVarLimitHeavyContactPush.GetValueOnGameThread() != 0 ? Body : nullptr);
    SetApproachTracking(CVarBrakeHeavyApproach.GetValueOnGameThread() != 0 ? Body : nullptr);
    if (!HeavyContacts.IsEmpty())
    {
        const double Cutoff = Owner.GetWorld()->GetTimeSeconds() - ContactExpirySeconds;
        HeavyContacts.RemoveAllSwap([Cutoff](const FHeavyContact& Contact)
        {
            return Contact.LastHitTime < Cutoff || !Contact.Body.IsValid();
        }, EAllowShrinking::No);
    }
}

void UCarryPhysicsComponent::ClearHeavyObstacleTracking()
{
    SetContactTracking(nullptr);
    SetApproachTracking(nullptr);
}

void UCarryPhysicsComponent::SetApproachTracking(UPrimitiveComponent* Body)
{
    if (TrackedApproachSourceBody.Get() == Body && !TrackedApproachSourceBody.IsStale())
    {
        return;
    }

    TrackedApproachSourceBody = Body;
    ApproachBody.Reset();
    NextApproachQuery = 0.0;
}

void UCarryPhysicsComponent::SetContactTracking(UPrimitiveComponent* Body)
{
    if (TrackedContactBody.Get() == Body && !TrackedContactBody.IsStale())
    {
        return;
    }

    if (UPrimitiveComponent* Previous = TrackedContactBody.Get())
    {
        Previous->OnComponentHit.RemoveDynamic(this, &UCarryPhysicsComponent::OnHeldBodyHit);
        if (bEnabledHitNotifications)
        {
            Previous->SetNotifyRigidBodyCollision(false);
        }
    }
    TrackedContactBody = Body;
    HeavyContacts.Reset();
    bEnabledHitNotifications = false;
    if (Body)
    {
        const FBodyInstance* Instance = Body->GetBodyInstance();
        bEnabledHitNotifications = Instance && !Instance->bNotifyRigidBodyCollision;
        Body->OnComponentHit.AddUniqueDynamic(this, &UCarryPhysicsComponent::OnHeldBodyHit);
        if (bEnabledHitNotifications)
        {
            Body->SetNotifyRigidBodyCollision(true);
        }
    }
}

void UCarryPhysicsComponent::OnHeldBodyHit(UPrimitiveComponent* HitComponent, AActor* OtherActor,
    UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit)
{
    CSV_SCOPED_TIMING_STAT(CarryContacts, OnHeldBodyHit);
    const FVector Normal = Hit.ImpactNormal.GetSafeNormal2D();
    if (HitComponent != TrackedContactBody.Get() || !IsValid(OtherActor) || !IsValid(OtherComponent)
        || OtherActor == GetOwner() || FMath::Abs(Hit.ImpactNormal.Z) > 0.5 || Normal.IsNearlyZero()
        || !OtherComponent->IsSimulatingPhysics(Hit.BoneName))
    {
        return;
    }

    const double Now = GetWorld()->GetTimeSeconds();
    FHeavyContact* Match = HeavyContacts.FindByPredicate([&](const FHeavyContact& Contact)
    {
        return Contact.Body == OtherComponent && FVector::DotProduct(Contact.OutwardNormal, Normal) > 0.95;
    });
    if (Match)
    {
        Match->OutwardNormal = Normal;
        Match->LastHitTime = Now;
        return;
    }

    const double OtherMass = OtherComponent->GetMass();
    if (OtherMass <= TrackedBodyMass * MinMassRatio || !OtherActor->FindComponentByClass<UCarryableComponent>())
    {
        return;
    }

    if (HeavyContacts.Num() < MaxHeavyContacts)
    {
        HeavyContacts.Add({ OtherComponent, Normal, OtherMass, Now });
        return;
    }
    FHeavyContact* Oldest = &HeavyContacts[0];
    for (FHeavyContact& Contact : HeavyContacts)
    {
        if (Contact.LastHitTime < Oldest->LastHitTime)
        {
            Oldest = &Contact;
        }
    }
    *Oldest = { OtherComponent, Normal, OtherMass, Now };
}

FVector UCarryPhysicsComponent::LimitContactTarget(const FVector& Target, const FVector& GripPoint, double AllowedError) const
{
    CSV_SCOPED_TIMING_STAT(CarryContacts, LimitContactTarget);
    FVector Error = Target - GripPoint;
    for (const FHeavyContact& Contact : HeavyContacts)
    {
        const double InwardError = -FVector::DotProduct(Error, Contact.OutwardNormal);
        if (InwardError > AllowedError)
        {
            const double Limit = FMath::SmoothStep(MinMassRatio, FullLimitMassRatio, Contact.OtherMass / TrackedBodyMass);
            Error += Contact.OutwardNormal * (InwardError - AllowedError) * Limit;
        }
    }
    return GripPoint + Error;
}

void UCarryPhysicsComponent::UpdateHeavyApproach(UPrimitiveComponent& Body, const UCarryableComponent& Item,
    const FVector& DriveError, double Now)
{
    if (Now < NextApproachQuery || DriveError.SizeSquared2D() < FMath::Square(ApproachDriveThreshold))
    {
        return;
    }
    NextApproachQuery = Now + ApproachQueryInterval;
    CSV_SCOPED_TIMING_STAT(CarryContacts, SweepHeavyApproach);

    const CarryWorld::FHeavyApproachHit Hit = CarryWorld::SweepHeavyApproach(*GetWorld(), GetOwner(), Body,
        Item.Grips, DriveError.GetSafeNormal2D(), ApproachDistance, TrackedBodyMass, MinMassRatio);
    if (Hit.Body)
    {
        ApproachBody = Hit.Body;
        ApproachNormal = Hit.Normal;
        ApproachStopOffset = Hit.StopOffset;
        ApproachMass = Hit.Mass;
        LastApproachTime = Now;
    }
}

FVector UCarryPhysicsComponent::LimitApproachTarget(const FVector& Target, const FVector& GripPoint,
    const FVector& BodyLocation, double AllowedError, double Now) const
{
    const UPrimitiveComponent* Other = ApproachBody.Get();
    if (!Other || Now - LastApproachTime > ApproachExpiry)
    {
        return Target;
    }
    const double Gap = FVector::DotProduct(BodyLocation - Other->GetComponentLocation() - ApproachStopOffset, ApproachNormal);
    const double InwardError = -FVector::DotProduct(Target - GripPoint, ApproachNormal);
    const double SafeError = FMath::Max(AllowedError, ApproachErrorFraction * Gap);
    if (InwardError <= SafeError)
    {
        return Target;
    }
    const double Limit = FMath::SmoothStep(MinMassRatio, FullLimitMassRatio, ApproachMass / TrackedBodyMass);
    return Target + ApproachNormal * (InwardError - SafeError) * Limit;
}

FVector UCarryPhysicsComponent::LimitJointTarget(const FVector& Target, UPrimitiveComponent& Body,
    const UCarryableComponent& Item, const FVector& BodyLocation, const FVector& BodyContact,
    double MassPerHolder, double LinearStiffnessChaos)
{
    if (HeavyContacts.IsEmpty() && !TrackedApproachSourceBody.IsValid())
    {
        return Target;
    }

    // Keep the camera target intact. Limit only the endpoint that drives the
    // joint, and only along the horizontal normal toward a heavy contact.
    const double AllowedError = HeavyContactAcceleration * Carry::CentimetersPerMeter * MassPerHolder / LinearStiffnessChaos;
    if (!HeavyContacts.IsEmpty())
    {
        return LimitContactTarget(Target, BodyContact, AllowedError);
    }
    const double Now = GetWorld()->GetTimeSeconds();
    UpdateHeavyApproach(Body, Item, Target - BodyContact, Now);
    return LimitApproachTarget(Target, BodyContact, BodyLocation, AllowedError, Now);
}
