#include "CarryableComponent.h"

#include "CarryProfileSettings.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/HitResult.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerState.h"
#include "PlayerCarryComponent.h"
#include "World/CarryView.h"
#include "World/CarryWorldQueries.h"

TArray<int32> UCarryableComponent::GetGripPlayerIds() const
{
    TArray<int32> PlayerIds;
    PlayerIds.Reserve(Grips.Num());
    for (const FCarryGrip& Grip : Grips)
    {
        PlayerIds.Add(Grip.PlayerId);
    }
    return PlayerIds;
}

bool UCarryableComponent::TryAddGrip(APawn& Holder, const FHitResult& Hit)
{
    AActor* Item = GetOwner();
    UPrimitiveComponent* Body = GetBodyComponent();
    if (!Item || !Item->HasAuthority() || !Item->GetIsReplicated() || !Body || Hit.GetComponent() != Body ||
        !Body->IsSimulatingPhysics() || Grips.Num() >= GetEffectiveMaxHolders() || HasGripFor(Holder))
    {
        return false;
    }

    if (!CarryWorld::EvaluateGripReach(*Item->GetWorld(), Holder, *Body, Hit.ImpactPoint, Hit.ImpactNormal).CanReach())
    {
        return false;
    }

    Item->FlushNetDormancy();
    Body->WakeAllRigidBodies();
    FCarryGrip& Grip = Grips.AddDefaulted_GetRef();
    Grip.Holder = &Holder;
    if (const APlayerState* PlayerState = Holder.GetPlayerState())
    {
        Grip.PlayerId = PlayerState->GetPlayerId();
    }
    InitializeGripAnchor(Grip, Holder, *Body, Hit);
    Item->ForceNetUpdate();
    OnRep_Grips();
    return true;
}

void UCarryableComponent::InitializeGripAnchor(FCarryGrip& Grip, const APawn& Holder, UPrimitiveComponent& Body,
    const FHitResult& Hit) const
{
    const FTransform& BodyTransform = Body.GetComponentTransform();
    Grip.LocalPoint = BodyTransform.InverseTransformPosition(Hit.ImpactPoint);
    Grip.LocalNormal = BodyTransform.InverseTransformVectorNoScale(Hit.ImpactNormal).GetSafeNormal();
    const FTransform View = GetCarryViewTransform(Holder);
    Grip.ViewLocalTarget = View.InverseTransformPosition(Hit.ImpactPoint);
    Grip.SmoothedTarget = Hit.ImpactPoint;
    const FVector InitialOffset = Hit.ImpactPoint - View.GetLocation();
    Grip.PreviousAimDirection = InitialOffset.GetSafeNormal(UE_SMALL_NUMBER, View.GetRotation().GetForwardVector());
    Grip.SmoothedHoldDistance = InitialOffset.Size();
    Grip.CarryTargetYaw = FMath::Atan2(InitialOffset.Y, InitialOffset.X);
    Grip.SmoothedHorizontalReach = InitialOffset.Size2D();
    Grip.CenterRadiusOffset = static_cast<float>((Body.GetCenterOfMass() - View.GetLocation()).Size2D() - Grip.SmoothedHorizontalReach);
    Grip.InitializedHolder = &Holder;
    Grip.InitializedLocalPoint = Grip.LocalPoint;
    Grip.LastRawViewLocation = View.GetLocation();
    Grip.LastRawViewRotation = View.GetRotation();
    Grip.RemoteViewAngularVelocity = FVector::ZeroVector;
    Grip.TimeSinceViewSample = 0.0;
    Grip.bStabilizeTurnDistance = false;
    Grip.bEaseOutFastTurn = false;
    Grip.RecentLookSpeed = 0.0;
    Grip.InitialBodyRotation = BodyTransform.GetRotation();
    Grip.InitialViewYaw = View.GetRotation().Rotator().Yaw;
    Grip.PreviousRotationInput = 0;
    Grip.RotationHoldDuration = 0.0;
}

void UCarryableComponent::UpdateAutoRegrip(const APawn& Holder, double Now)
{
    AActor* Item = GetOwner();
    UPrimitiveComponent* Body = GetBodyComponent();
    if (!Item || !Item->HasAuthority() || !Body || !Body->IsSimulatingPhysics()
        || Grips.Num() != 1 || Grips[0].Holder != &Holder)
    {
        return;
    }

    constexpr double ReachCheckInterval = 0.10;
    constexpr double MinGripChange = 5.0;
    FCarryGrip& Grip = Grips[0];
    if (Now < Grip.NextReachCheck)
    {
        return;
    }
    Grip.NextReachCheck = Now + ReachCheckInterval;

    UWorld& World = *Item->GetWorld();
    const FTransform& BodyTransform = Body->GetComponentTransform();
    const FVector Contact = BodyTransform.TransformPosition(Grip.LocalPoint);
    const FVector Normal = BodyTransform.TransformVectorNoScale(Grip.LocalNormal).GetSafeNormal();
    const CarryWorld::FGripReach Reach = CarryWorld::EvaluateGripReach(World, Holder, *Body, Contact, Normal);
    if (!Reach.IsBlocked())
    {
        UpdateClearRegrip(Grip, Reach.CanReach(), Now);
        return;
    }

    Grip.ClearReachSince = -1.0;
    if (!Grip.bSuppressSwing)
    {
        Grip.SwingHoldRotation = BodyTransform.GetRotation();
        Grip.bSuppressSwing = true;
    }
    if (Grip.bRegrippedWhileBlocked)
    {
        return;
    }

    static const FCarryPlayerSettings Defaults;
    const UPlayerCarryComponent* PlayerCarry = Holder.FindComponentByClass<UPlayerCarryComponent>();
    const FCarryPlayerSettings* Profile = PlayerCarry ? PlayerCarry->GetProfileSettings() : nullptr;
    const FCarryPlayerSettings& Settings = Profile ? *Profile : Defaults;
    const FTransform View = GetCarryViewTransform(Holder);
    FHitResult Hit;
    if (!CarryWorld::FindRegripContact(World, Holder, *Body, View, Reach.Start, Contact, Settings, MinGripChange, Hit))
    {
        return;
    }

    ApplyRegrip(*Item, Grip, Holder, *Body, Hit, Now);
}

void UCarryableComponent::UpdateClearRegrip(FCarryGrip& Grip, bool bCanReach, double Now)
{
    constexpr double ClearGrace = 0.20;
    if (!Grip.bSuppressSwing || !bCanReach)
    {
        Grip.ClearReachSince = -1.0;
        return;
    }
    if (Grip.ClearReachSince < 0.0)
    {
        Grip.ClearReachSince = Now;
    }
    if (Now - Grip.ClearReachSince >= ClearGrace)
    {
        Grip.bSuppressSwing = false;
        Grip.bRegrippedWhileBlocked = false;
    }
}

void UCarryableComponent::ApplyRegrip(AActor& Item, FCarryGrip& Grip, const APawn& Holder, UPrimitiveComponent& Body,
    const FHitResult& Hit, double Now)
{
    const double DesiredDistance = Grip.ViewLocalTarget.Size();
    InitializeGripAnchor(Grip, Holder, Body, Hit);
    Grip.ViewLocalTarget = Grip.ViewLocalTarget.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector) * DesiredDistance;
    Grip.RegripTime = Now;
    Grip.bRegrippedWhileBlocked = true;
    Item.FlushNetDormancy();
    Item.ForceNetUpdate();
    OnRep_Grips();
}

void UCarryableComponent::RemoveGrip(const APawn& Holder)
{
    AActor* Item = GetOwner();
    if (!Item || !Item->HasAuthority())
    {
        return;
    }

    const int32 Index = Grips.IndexOfByPredicate([&Holder](const FCarryGrip& Grip) { return Grip.Holder == &Holder; });
    if (Index != INDEX_NONE)
    {
        Item->FlushNetDormancy();
        Grips.RemoveAt(Index);
        Item->ForceNetUpdate();
        OnRep_Grips();
    }
}

FCarryGrip* UCarryableComponent::FindGripFor(const APawn& Holder)
{
    return Grips.FindByPredicate([&Holder](const FCarryGrip& Grip) { return Grip.Holder == &Holder; });
}

void UCarryableComponent::SetSoloGripRotationInput(const APawn& Holder, int32 Direction)
{
    if (Grips.Num() == 1 && Grips[0].Holder == &Holder)
    {
        Grips[0].RotationInput = FMath::Clamp(Direction, -1, 1);
    }
}

bool UCarryableComponent::HasGripFor(const APawn& Holder) const
{
    return Grips.ContainsByPredicate([&Holder](const FCarryGrip& Grip) { return Grip.Holder == &Holder; });
}

bool UCarryableComponent::TryGetGripWorldPoint(const APawn& Holder, FVector& OutPoint) const
{
    const UPrimitiveComponent* Body = GetBodyComponent();
    const FCarryGrip* Grip = Grips.FindByPredicate([&Holder](const FCarryGrip& Entry) { return Entry.Holder == &Holder; });
    if (!IsValid(Body) || !Body->IsSimulatingPhysics() || !Grip)
    {
        return false;
    }

    OutPoint = Body->GetComponentTransform().TransformPosition(Grip->LocalPoint);
    return true;
}

void UCarryableComponent::RemoveInvalidGrips()
{
    AActor* Item = GetOwner();
    if (!Item || !Item->HasAuthority())
    {
        return;
    }

    const int32 Removed = Grips.RemoveAll([](const FCarryGrip& Grip) { return !IsValid(Grip.Holder.Get()); });
    if (Removed > 0)
    {
        Item->FlushNetDormancy();
        Item->ForceNetUpdate();
        OnRep_Grips();
    }
}
