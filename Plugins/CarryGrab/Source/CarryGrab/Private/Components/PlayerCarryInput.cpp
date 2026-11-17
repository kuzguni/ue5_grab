#include "PlayerCarryComponent.h"

#include "Carry/CarryConstants.h"
#include "CarryableComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "World/CarryView.h"
#include "World/CarryWorldQueries.h"

namespace
{
    constexpr double TapThrowFraction = 0.2;
    constexpr float MinimumChargeTimeSeconds = 0.05f;
}

void UPlayerCarryComponent::RequestGrab()
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (Pawn && Pawn->IsLocallyControlled())
    {
        ServerRequestGrab();
    }
}

void UPlayerCarryComponent::RequestRelease()
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (Pawn && Pawn->IsLocallyControlled())
    {
        ServerRequestRelease();
    }
}

void UPlayerCarryComponent::SetThrowChargeInput(bool bPressed, bool bCanceled)
{
    if (const APawn* Pawn = Cast<APawn>(GetOwner()); Pawn && Pawn->IsLocallyControlled())
    {
        ServerSetThrowChargeInput(bPressed, bCanceled);
    }
}

void UPlayerCarryComponent::ServerSetThrowChargeInput_Implementation(bool bPressed, bool bCanceled)
{
    if (bCanceled)
    {
        ThrowChargeStarted = -1.0;
        return;
    }

    const APawn* Pawn = Cast<APawn>(GetOwner());
    UCarryableComponent* Item = HeldItem.Get();
    if (!Pawn || !Pawn->IsPlayerControlled() || !IsValid(Item) || Item->GetGripCount() != 1 || !Item->HasGripFor(*Pawn))
    {
        ThrowChargeStarted = -1.0;
        return;
    }

    const double Now = GetWorld()->GetTimeSeconds();
    if (bPressed)
    {
        if (ThrowChargeStarted < 0.0)
        {
            ThrowChargeStarted = Now;
        }
        return;
    }

    const double Started = ThrowChargeStarted;
    ThrowChargeStarted = -1.0;
    UPrimitiveComponent* Body = Item->GetBodyComponent();
    if (Started < 0.0 || !IsValid(Body) || !Body->IsSimulatingPhysics())
    {
        return;
    }

    const double Charge = FMath::Clamp((Now - Started) / FMath::Max(MinimumChargeTimeSeconds, ThrowChargeSeconds), 0.0, 1.0);
    const double FullImpulse = FMath::Min(Carry::CentimetersPerMeter * FMath::Max(0.0f, MaxThrowImpulse),
        static_cast<double>(Body->GetMass()) * FMath::Max(0.0f, MaxThrowAddedSpeed));
    const FVector Impulse = GetCarryViewTransform(*Pawn).GetRotation().GetForwardVector()
        * (FullImpulse * FMath::Lerp(TapThrowFraction, 1.0, Charge));

    ReleaseGrip();
    if (IsValid(Body) && IsValid(Item) && Item->GetGripCount() == 0 && Body->IsSimulatingPhysics())
    {
        Body->AddImpulse(Impulse);
        if (AActor* Owner = Body->GetOwner())
        {
            Owner->ForceNetUpdate();
        }
    }
}

void UPlayerCarryComponent::AdjustHoldDistance(float Direction)
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (Pawn && Pawn->IsLocallyControlled() && FMath::IsFinite(Direction) && !FMath::IsNearlyZero(Direction))
    {
        ServerAdjustHoldDistance(Direction > 0.0f ? 1 : -1);
    }
}

void UPlayerCarryComponent::SetCarryRotationInput(float Direction)
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (Pawn && Pawn->IsLocallyControlled() && FMath::IsFinite(Direction))
    {
        const int32 Input = FMath::IsNearlyZero(Direction) ? 0 : (Direction > 0.0f ? 1 : -1);
        if (Input != LocalRotationInput)
        {
            LocalRotationInput = Input;
            ServerSetCarryRotationInput(Input);
        }
    }
}

void UPlayerCarryComponent::ServerAdjustHoldDistance_Implementation(int32 Direction)
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    UCarryableComponent* Item = HeldItem.Get();
    FCarryGrip* Grip = Item && Pawn ? Item->FindGripFor(*Pawn) : nullptr;
    if (!Grip || (Direction != -1 && Direction != 1))
    {
        return;
    }

    const FCarryPlayerSettings* Settings = GetProfileSettings();
    const double MaxDistance = FMath::Max(1.0, static_cast<double>(Settings ? Settings->PickupReach : MaxReach));
    const double MinDistance = FMath::Clamp(static_cast<double>(Settings ? Settings->MinimumHoldDistance : 55.0f), 1.0, MaxDistance);
    const double Step = FMath::Max(1.0, static_cast<double>(Settings ? Settings->HoldDistanceStep : 15.0f));
    const double Distance = Grip->ViewLocalTarget.Size();
    const double NewDistance = FMath::Clamp(Distance + Direction * Step, FMath::Min(Distance, MinDistance), MaxDistance);
    if (!FMath::IsNearlyEqual(NewDistance, Distance))
    {
        const FVector Aim = Grip->ViewLocalTarget.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
        Grip->ViewLocalTarget = Aim * NewDistance;
        Item->GetOwner()->ForceNetUpdate();
    }
}

void UPlayerCarryComponent::ServerSetCarryRotationInput_Implementation(int32 Direction)
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    UCarryableComponent* Item = HeldItem.Get();
    if (Item && Pawn)
    {
        Item->SetSoloGripRotationInput(*Pawn, Direction);
    }
}

void UPlayerCarryComponent::ServerRequestGrab_Implementation()
{
    APawn* Pawn = Cast<APawn>(GetOwner());
    if (!Pawn || !Pawn->IsPlayerControlled())
    {
        return;
    }
    if (UCarryableComponent* ExistingItem = HeldItem.Get())
    {
        if (ExistingItem->HasGripFor(*Pawn))
        {
            return;
        }
        HeldItem = nullptr;
    }

    const FTransform View = GetCarryViewTransform(*Pawn);
    const FCarryPlayerSettings* Settings = GetProfileSettings();
    FHitResult Hit;
    if (!CarryWorld::TracePickup(*GetWorld(), *Pawn, View, Settings ? Settings->PickupReach : MaxReach, Hit))
    {
        return;
    }

    UCarryableComponent* Item = Hit.GetActor()->FindComponentByClass<UCarryableComponent>();
    if (Item && Item->TryAddGrip(*Pawn, Hit))
    {
        ThrowChargeStarted = -1.0;
        HeldItem = Item;
        OutOfReachSince = -1.0;
        RefreshGripForce(*Pawn, 0.0f);
        SetComponentTickEnabled(true);
        OnItemGrabbed.Broadcast(Item);
    }
}

void UPlayerCarryComponent::ServerRequestRelease_Implementation()
{
    ReleaseGrip();
}
