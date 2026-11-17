#include "PlayerCarryComponent.h"

#include "CarryableComponent.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "Net/UnrealNetwork.h"

UPlayerCarryComponent::UPlayerCarryComponent()
{
    SetIsReplicatedByDefault(true);
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UPlayerCarryComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(UPlayerCarryComponent, HeldItem);
    DOREPLIFETIME_CONDITION(UPlayerCarryComponent, CurrentGripForceNewtons, COND_OwnerOnly);
}

void UPlayerCarryComponent::OnRep_HeldItem(UCarryableComponent* PreviousItem)
{
    if (PreviousItem && PreviousItem != HeldItem.Get())
    {
        OnItemReleased.Broadcast(PreviousItem);
    }
    const bool bHolding = IsValid(HeldItem.Get());
    if (!bHolding)
    {
        CurrentGripForceNewtons = FVector::ZeroVector;
        SmoothedGripForceNewtons = FVector::ZeroVector;
    }
    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (Pawn && !Pawn->HasAuthority())
    {
        SetComponentTickEnabled(bHolding && Pawn->IsLocallyControlled());
    }
    if (bHolding && HeldItem.Get() != PreviousItem)
    {
        OnItemGrabbed.Broadcast(HeldItem.Get());
    }
}

void UPlayerCarryComponent::BeginPlay()
{
    Super::BeginPlay();
    if (APawn* Pawn = Cast<APawn>(GetOwner()); Pawn && Pawn->HasAuthority())
    {
        Pawn->ReceiveControllerChangedDelegate.AddDynamic(this, &UPlayerCarryComponent::HandleControllerChanged);
    }
}

void UPlayerCarryComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (APawn* Pawn = Cast<APawn>(GetOwner()); Pawn && Pawn->HasAuthority())
    {
        Pawn->ReceiveControllerChangedDelegate.RemoveDynamic(this, &UPlayerCarryComponent::HandleControllerChanged);
        ReleaseGrip();
    }
    Super::EndPlay(EndPlayReason);
}

void UPlayerCarryComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (!Pawn)
    {
        SetComponentTickEnabled(false);
        return;
    }
    if (!Pawn->HasAuthority())
    {
        if (IsValid(HeldItem.Get()) && Pawn->IsLocallyControlled())
        {
            UpdateSmoothedForce(DeltaTime);
        }
        else
        {
            SmoothedGripForceNewtons = FVector::ZeroVector;
            SetComponentTickEnabled(false);
        }
        return;
    }

    UCarryableComponent* Item = HeldItem.Get();
    FVector GripPoint;
    if (!Item || !Item->TryGetGripWorldPoint(*Pawn, GripPoint))
    {
        ReleaseGrip();
        return;
    }
    if (ThrowChargeStarted >= 0.0 && Item->GetGripCount() != 1)
    {
        ThrowChargeStarted = -1.0;
    }

    const FCarryPlayerSettings* Settings = GetProfileSettings();
    const double HeldReach = Settings ? Settings->PickupReach + Settings->HeldReachSlack : MaxReach + HeldReachSlack;
    if (FVector::DistSquared(Pawn->GetPawnViewLocation(), GripPoint) <= FMath::Square(HeldReach))
    {
        OutOfReachSince = -1.0;
        if (bAutoRegrip)
        {
            Item->UpdateAutoRegrip(*Pawn, GetWorld()->GetTimeSeconds());
        }
        RefreshGripForce(*Pawn, DeltaTime);
        return;
    }

    const double Now = GetWorld()->GetTimeSeconds();
    if (OutOfReachSince < 0.0)
    {
        OutOfReachSince = Now;
    }
    if (Now - OutOfReachSince >= (Settings ? Settings->ReleaseGrace : HeldReachGraceSeconds))
    {
        ReleaseGrip();
        return;
    }

    RefreshGripForce(*Pawn, DeltaTime);
}

void UPlayerCarryComponent::HandleControllerChanged(APawn* Pawn, AController* OldController, AController* NewController)
{
    if (OldController && OldController != NewController)
    {
        ReleaseGrip();
    }
}

void UPlayerCarryComponent::ReleaseGrip()
{
    ThrowChargeStarted = -1.0;
    APawn* Pawn = Cast<APawn>(GetOwner());
    UCarryableComponent* ReleasedItem = HeldItem.Get();
    if (ReleasedItem && Pawn)
    {
        ReleasedItem->RemoveGrip(*Pawn);
    }
    HeldItem = nullptr;
    CurrentGripForceNewtons = FVector::ZeroVector;
    SmoothedGripForceNewtons = FVector::ZeroVector;
    OutOfReachSince = -1.0;
    SetComponentTickEnabled(false);
    if (ReleasedItem)
    {
        OnItemReleased.Broadcast(ReleasedItem);
    }
}
