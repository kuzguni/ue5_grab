#include "PlayerCarryComponent.h"

#include "CarryableComponent.h"
#include "CarryProfileBinding.h"
#include "CarryProfileSettings.h"
#include "CarryView.h"
#include "CollisionQueryParams.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"

UPlayerCarryComponent::UPlayerCarryComponent()
{
    SetIsReplicatedByDefault(true);
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

const FCarryPlayerSettings* UPlayerCarryComponent::GetProfileSettings() const
{
    return ReadCarryProfile<FCarryPlayerSettings>(PlayerProfile.Get());
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

void UPlayerCarryComponent::RequestGrab()
{
    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (Pawn && Pawn->IsLocallyControlled())
    {
        ServerRequestGrab();
    }
}

void UPlayerCarryComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    const APawn* Pawn = Cast<APawn>(GetOwner());
    if (!Pawn || !Pawn->HasAuthority())
    {
        SetComponentTickEnabled(false);
        return;
    }

    UCarryableComponent* Item = HeldItem.Get();
    FVector GripPoint;
    if (!Item || !Item->TryGetGripWorldPoint(*Pawn, GripPoint))
    {
        ReleaseGrip();
        return;
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
    FCarryGrip* Grip = Item && Pawn ? Item->Grips.FindByPredicate([Pawn](const FCarryGrip& Entry) { return Entry.Holder == Pawn; }) : nullptr;
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
    if (Item && Pawn && Item->Grips.Num() == 1 && Item->Grips[0].Holder == Pawn)
    {
        Item->Grips[0].RotationInput = FMath::Clamp(Direction, -1, 1);
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
        HeldItem.Reset();
    }

    const FTransform View = GetCarryViewTransform(*Pawn);
    const FVector Start = View.GetLocation();
    const FCarryPlayerSettings* Settings = GetProfileSettings();
    const FVector End = View.TransformPosition(FVector(Settings ? Settings->PickupReach : MaxReach, 0.0, 0.0));
    const FCollisionQueryParams Query(SCENE_QUERY_STAT(CarryGrab), false, Pawn);
    FHitResult Hit;
    if (!GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Query) || !Hit.GetActor())
    {
        return;
    }

    UCarryableComponent* Item = Hit.GetActor()->FindComponentByClass<UCarryableComponent>();
    if (Item && Item->TryAddGrip(*Pawn, Hit))
    {
        HeldItem = Item;
        OutOfReachSince = -1.0;
        SetComponentTickEnabled(true);
    }
}

void UPlayerCarryComponent::ServerRequestRelease_Implementation()
{
    ReleaseGrip();
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
    APawn* Pawn = Cast<APawn>(GetOwner());
    if (UCarryableComponent* Item = HeldItem.Get(); Item && Pawn)
    {
        Item->RemoveGrip(*Pawn);
    }
    HeldItem.Reset();
    OutOfReachSince = -1.0;
    SetComponentTickEnabled(false);
}
