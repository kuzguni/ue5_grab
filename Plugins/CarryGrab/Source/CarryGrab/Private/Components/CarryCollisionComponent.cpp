#include "CarryCollisionComponent.h"

#include "CarryableComponent.h"
#include "World/CarryWorldQueries.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Pawn.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

UCarryCollisionComponent::UCarryCollisionComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UCarryCollisionComponent::BeginPlay()
{
    Super::BeginPlay();
    if (const AActor* Owner = GetOwner())
    {
        CachedItemBody = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    }
}

bool UCarryCollisionComponent::IsHolderCollisionIgnored(APawn* Holder) const
{
    const FHolderCollision* Pair = Pairs.Find(Holder);
    const UPhysicsConstraintComponent* Constraint = Pair ? Pair->Constraint.Get() : nullptr;
    return Constraint && Constraint->ConstraintInstance.IsValidConstraintInstance();
}

void UCarryCollisionComponent::UpdateHolders(const TArray<FCarryGrip>& Grips)
{
    for (auto& [Holder, Pair] : Pairs)
    {
        Pair.bPendingRestore = true;
    }

    for (const FCarryGrip& Grip : Grips)
    {
        APawn* Holder = Grip.Holder.Get();
        if (!IsValid(Holder))
        {
            continue;
        }

        if (FHolderCollision* Pair = Pairs.Find(Holder))
        {
            Pair->bPendingRestore = false;
        }
        else
        {
            FHolderCollision NewPair;
            if (IgnoreHolder(*Holder, NewPair))
            {
                Pairs.Add(Holder, NewPair);
            }
        }
    }

    RestoreSeparatedHolders();
}

bool UCarryCollisionComponent::IgnoreHolder(APawn& Holder, FHolderCollision& Pair)
{
    AActor* Item = GetOwner();
    UPrimitiveComponent* ItemBody = CachedItemBody ? CachedItemBody.Get() : (Item ? Cast<UPrimitiveComponent>(Item->GetRootComponent()) : nullptr);
    UPrimitiveComponent* HolderBody = Cast<UPrimitiveComponent>(Holder.GetRootComponent());
    if (!ItemBody || !HolderBody || !ItemBody->IsPhysicsStateCreated() || !HolderBody->IsPhysicsStateCreated())
    {
        return false;
    }

    // A free Chaos constraint supplies a pair filter without driving or limiting motion.
    // Character movement sweeps also need their own per-actor ignore.
    UPhysicsConstraintComponent* Constraint = NewObject<UPhysicsConstraintComponent>(Item, NAME_None, RF_Transient);
    Constraint->SetLinearXLimit(LCM_Free, 0.0f);
    Constraint->SetLinearYLimit(LCM_Free, 0.0f);
    Constraint->SetLinearZLimit(LCM_Free, 0.0f);
    Constraint->SetAngularSwing1Limit(ACM_Free, 0.0f);
    Constraint->SetAngularSwing2Limit(ACM_Free, 0.0f);
    Constraint->SetAngularTwistLimit(ACM_Free, 0.0f);
    Constraint->SetDisableCollision(true);
    Constraint->RegisterComponent();
    Constraint->SetConstrainedComponents(ItemBody, NAME_None, HolderBody, NAME_None);

    if (!Constraint->ConstraintInstance.IsValidConstraintInstance())
    {
        Constraint->DestroyComponent();
        return false;
    }

    Pair.Constraint = Constraint;
    Pair.bAddedMoveIgnore = !HolderBody->GetMoveIgnoreActors().Contains(Item);
    if (Pair.bAddedMoveIgnore)
    {
        HolderBody->IgnoreActorWhenMoving(Item, true);
    }
    return true;
}

void UCarryCollisionComponent::RestoreHolder(APawn* Holder, const FHolderCollision& Pair)
{
    if (UPhysicsConstraintComponent* Constraint = Pair.Constraint.Get())
    {
        Constraint->DestroyComponent();
    }
    if (IsValid(Holder) && Pair.bAddedMoveIgnore)
    {
        if (UPrimitiveComponent* HolderBody = Cast<UPrimitiveComponent>(Holder->GetRootComponent()))
        {
            HolderBody->IgnoreActorWhenMoving(GetOwner(), false);
        }
    }
}

void UCarryCollisionComponent::RestoreSeparatedHolders()
{
    UPrimitiveComponent* ItemBody = CachedItemBody ? CachedItemBody.Get() : (GetOwner() ? Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent()) : nullptr);
    bool bWaitingForSeparation = false;
    for (auto It = Pairs.CreateIterator(); It; ++It)
    {
        APawn* Holder = It.Key().Get();
        const FHolderCollision& Pair = It.Value();
        if (!Holder || !Pair.Constraint.IsValid())
        {
            RestoreHolder(Holder, Pair);
            It.RemoveCurrent();
            continue;
        }
        if (!Pair.bPendingRestore)
        {
            continue;
        }

        UPrimitiveComponent* HolderBody = Cast<UPrimitiveComponent>(Holder->GetRootComponent());
        // Restoring contacts inside the capsule would let the solver eject the prop.
        if (ItemBody && HolderBody && CarryWorld::OverlapsHolder(*ItemBody, *HolderBody))
        {
            bWaitingForSeparation = true;
            continue;
        }

        RestoreHolder(Holder, Pair);
        It.RemoveCurrent();
    }
    SetComponentTickEnabled(bWaitingForSeparation);
}

void UCarryCollisionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    RestoreSeparatedHolders();
}

void UCarryCollisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    for (const auto& [Holder, Pair] : Pairs)
    {
        RestoreHolder(Holder.Get(), Pair);
    }
    Pairs.Reset();
    Super::EndPlay(EndPlayReason);
}
