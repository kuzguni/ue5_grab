#pragma once

#include "Components/ActorComponent.h"
#include "CarryCollisionComponent.generated.h"

class APawn;
class UPhysicsConstraintComponent;
class UPrimitiveComponent;
struct FCarryGrip;

/** Ignores contacts only between this prop and its current holders. */
UCLASS(ClassGroup = (Carry), meta = (BlueprintSpawnableComponent))
class CARRYGRAB_API UCarryCollisionComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UCarryCollisionComponent();

    /** Also true during the overlap grace period after a holder releases. */
    UFUNCTION(BlueprintPure, Category = "Carry|Collision")
    bool IsHolderCollisionIgnored(APawn* Holder) const;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    friend class UCarryableComponent;

    struct FHolderCollision
    {
        TWeakObjectPtr<UPhysicsConstraintComponent> Constraint;
        TWeakObjectPtr<UPrimitiveComponent> IgnoredComponent;
        bool bAddedMoveIgnore = false;
        bool bPendingRestore = false;
    };

    UPrimitiveComponent* GetItemBody() const;
    void UpdateHolders(const TArray<FCarryGrip>& Grips);
    bool IgnoreHolder(APawn& Holder, FHolderCollision& Pair);
    void RestoreHolder(APawn* Holder, const FHolderCollision& Pair);
    void RestoreSeparatedHolders();

    TMap<TWeakObjectPtr<APawn>, FHolderCollision> Pairs;

    UPROPERTY(Transient)
    TObjectPtr<UPrimitiveComponent> CachedItemBody = nullptr;
};
