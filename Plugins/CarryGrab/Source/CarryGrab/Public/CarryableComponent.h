#pragma once

#include "Components/ActorComponent.h"
#include "Engine/NetSerialization.h"
#include "CarryableComponent.generated.h"

class APawn;
class UCarryCollisionComponent;
class UCarryPhysicsComponent;
class UPlayerCarryComponent;
class UPrimitiveComponent;
class UPrimaryDataAsset;
struct FCarryItemSettings;

USTRUCT()
struct FCarryGrip
{
    GENERATED_BODY()

    UPROPERTY()
    TObjectPtr<APawn> Holder = nullptr;

    UPROPERTY()
    int32 PlayerId = INDEX_NONE;

    UPROPERTY()
    FVector_NetQuantize10 LocalPoint = FVector::ZeroVector;

    UPROPERTY()
    FVector_NetQuantizeNormal LocalNormal = FVector::UpVector;

    UPROPERTY()
    FVector_NetQuantize10 ViewLocalTarget = FVector::ZeroVector;

    UPROPERTY()
    float CenterRadiusOffset = 0.0f;

    // Local controller state. Not replicated; initialized per world from the fields above.
    TWeakObjectPtr<const APawn> InitializedHolder = nullptr;
    FVector InitializedLocalPoint = FVector::ZeroVector;
    FVector SmoothedTarget = FVector::ZeroVector;
    FVector PreviousAimDirection = FVector::ForwardVector;
    double SmoothedHoldDistance = 0.0;
    FVector LastRawViewLocation = FVector::ZeroVector;
    FQuat LastRawViewRotation = FQuat::Identity;
    FVector RemoteViewAngularVelocity = FVector::ZeroVector;
    double TimeSinceViewSample = 0.0;
    double CarryTargetYaw = 0.0;
    double SmoothedHorizontalReach = 0.0;
    bool bStabilizeTurnDistance = false;
    bool bEaseOutFastTurn = false;
    double RecentLookSpeed = 0.0;
    FVector AppliedForce = FVector::ZeroVector;
    FQuat InitialBodyRotation = FQuat::Identity;
    double InitialViewYaw = 0.0;
    int32 RotationInput = 0;
    int32 PreviousRotationInput = 0;
    double RotationHoldDuration = 0.0;
    bool bHasManualYawTarget = false;
    FQuat ManualYawTarget = FQuat::Identity;
    FQuat SwingHoldRotation = FQuat::Identity;
    double NextReachCheck = 0.0;
    double ClearReachSince = -1.0;
    double RegripTime = -1.0;
    double SwingHoldAlpha = 0.0;
    bool bSuppressSwing = false;
    bool bRegrippedWhileBlocked = false;
};

/** Owns a carryable item's grip membership and reconciles its physics and collision components. */
UCLASS(ClassGroup = (Carry), meta = (BlueprintSpawnableComponent))
class CARRYGRAB_API UCarryableComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UCarryableComponent();
    const FCarryItemSettings* GetProfileSettings() const;

    UFUNCTION(BlueprintPure, Category = "Carry")
    int32 GetGripCount() const { return Grips.Num(); }

    /** Effective limit after applying this item's instance override and profile. */
    UFUNCTION(BlueprintPure, Category = "Carry")
    int32 GetEffectiveMaxHolders() const;

    UFUNCTION(BlueprintPure, Category = "Carry")
    TArray<int32> GetGripPlayerIds() const;

    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    friend class UCarryPhysicsComponent;
    friend class UPlayerCarryComponent;

    bool TryAddGrip(APawn& Holder, const FHitResult& Hit);
    void UpdateAutoRegrip(const APawn& Holder, double Now);
    static void UpdateClearRegrip(FCarryGrip& Grip, bool bCanReach, double Now);
    void ApplyRegrip(AActor& Item, FCarryGrip& Grip, const APawn& Holder, UPrimitiveComponent& Body, const FHitResult& Hit, double Now);
    void InitializeGripAnchor(FCarryGrip& Grip, const APawn& Holder, UPrimitiveComponent& Body, const FHitResult& Hit) const;
    void RemoveGrip(const APawn& Holder);
    FCarryGrip* FindGripFor(const APawn& Holder);
    void SetSoloGripRotationInput(const APawn& Holder, int32 Direction);
    bool HasGripFor(const APawn& Holder) const;
    bool TryGetGripWorldPoint(const APawn& Holder, FVector& OutPoint) const;
    void RemoveInvalidGrips();
    UPrimitiveComponent* GetBodyComponent() const;

    UFUNCTION()
    void OnRep_Grips();

    UPROPERTY(ReplicatedUsing = OnRep_Grips)
    TArray<FCarryGrip> Grips;

    /** Assign this item's Blueprint profile in the level or Construction Script. Body overrides apply at Begin Play. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Profile", meta = (AllowPrivateAccess = "true"))
    TObjectPtr<UPrimaryDataAsset> ItemProfile;

    /** This item's holder limit. Use 0 to inherit the profile, or the fallback if no profile is assigned. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Profile", meta = (AllowPrivateAccess = "true", ClampMin = "0", ClampMax = "4"))
    int32 MaxHoldersOverride = 0;

    /** Holder limit when neither a profile nor an instance override is set. */
    UPROPERTY(EditAnywhere, Category = "Carry|Fallbacks", meta = (ClampMin = "1", ClampMax = "4", EditCondition = "ItemProfile == nullptr && MaxHoldersOverride == 0", EditConditionHides))
    int32 MaxHolders = 4;

    UPROPERTY(EditAnywhere, Category = "Carry|Debug")
    bool bShowGripMarkers = true;

    /** Draw the two center sweeps from elbow height to the held grab point. */
    UPROPERTY(EditAnywhere, Category = "Carry|Debug")
    bool bShowBentReachPreview = true;

    UPROPERTY(Transient)
    TObjectPtr<UPrimitiveComponent> CachedBody = nullptr;

    UPROPERTY(Transient)
    TObjectPtr<UCarryPhysicsComponent> CachedPhysics = nullptr;

    UPROPERTY(Transient)
    TObjectPtr<UCarryCollisionComponent> CachedCollision = nullptr;
};
