#include "PlayerCarryComponent.h"

#include "CarryableComponent.h"
#include "CarryPhysicsComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Data/CarryProfileBinding.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"

const FCarryPlayerSettings* UPlayerCarryComponent::GetProfileSettings() const
{
    return ReadCarryProfile<FCarryPlayerSettings>(PlayerProfile.Get());
}

bool UPlayerCarryComponent::IsHoldingItem() const
{
    return IsValid(HeldItem.Get());
}

float UPlayerCarryComponent::GetHeldItemMassKg() const
{
    const UCarryableComponent* Item = HeldItem.Get();
    const UPrimitiveComponent* Body = Item ? Item->GetBodyComponent() : nullptr;
    return (Body && Body->IsSimulatingPhysics()) ? Body->GetMass() : 0.0f;
}

float UPlayerCarryComponent::GetMaxForceNewtons() const
{
    const FCarryPlayerSettings* Settings = GetProfileSettings();
    return Settings ? FMath::Max(0.0f, Settings->MaxForceNewtons) : 700.0f;
}

float UPlayerCarryComponent::GetGripEffortRatio() const
{
    const float MaxForce = GetMaxForceNewtons();
    if (MaxForce <= UE_SMALL_NUMBER)
    {
        return 0.0f;
    }
    return FMath::Clamp(GetGripForceMagnitudeNewtons() / MaxForce, 0.0f, 1.0f);
}

float UPlayerCarryComponent::GetSmoothedGripEffortRatio() const
{
    const float MaxForce = GetMaxForceNewtons();
    if (MaxForce <= UE_SMALL_NUMBER)
    {
        return 0.0f;
    }
    return FMath::Clamp(GetSmoothedGripForceMagnitudeNewtons() / MaxForce, 0.0f, 1.0f);
}

FCarryPlayerSettings UPlayerCarryComponent::GetEffectivePlayerSettings() const
{
    FCarryPlayerSettings Effective;
    if (const FCarryPlayerSettings* Settings = GetProfileSettings())
    {
        Effective = *Settings;
    }
    else
    {
        Effective.PickupReach = MaxReach;
        Effective.HeldReachSlack = HeldReachSlack;
        Effective.ReleaseGrace = HeldReachGraceSeconds;
    }
    Effective.RotationSpeedDegrees = FMath::Max(0.0f, RotationStartSpeedDegrees);
    return Effective;
}

void UPlayerCarryComponent::UpdateSmoothedForce(float DeltaTime)
{
    if (DeltaTime <= UE_SMALL_NUMBER)
    {
        return;
    }
    const FVector TargetForce(CurrentGripForceNewtons);
    const bool bRising = TargetForce.SizeSquared() > SmoothedGripForceNewtons.SizeSquared();
    const double Rate = FMath::Max(0.5, static_cast<double>(bRising ? ForceRiseSmoothingRate : ForceFallSmoothingRate));
    const double Alpha = 1.0 - FMath::Exp(-Rate * static_cast<double>(DeltaTime));
    SmoothedGripForceNewtons = FMath::Lerp(SmoothedGripForceNewtons, TargetForce, Alpha);
}

void UPlayerCarryComponent::RefreshGripForce(const APawn& Pawn, float DeltaTime)
{
    const UCarryableComponent* Item = HeldItem.Get();
    const AActor* ItemOwner = Item ? Item->GetOwner() : nullptr;
    const UCarryPhysicsComponent* Physics = ItemOwner ? ItemOwner->FindComponentByClass<UCarryPhysicsComponent>() : nullptr;
    CurrentGripForceNewtons = Physics ? Physics->GetGripForceNewtons(&Pawn) : FVector::ZeroVector;
    UpdateSmoothedForce(DeltaTime);
}
