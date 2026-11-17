#pragma once

#include "Components/ActorComponent.h"
#include "CarryProfileSettings.h"
#include "Engine/NetSerialization.h"
#include "PlayerCarryComponent.generated.h"

class AController;
class APawn;
class UCarryableComponent;
class UPrimaryDataAsset;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FCarryItemChangedSignature, UCarryableComponent*, Item);

/** Receives carry input for a pawn and exposes its held item and grip effort to Blueprints. */
UCLASS(ClassGroup = (Carry), meta = (BlueprintSpawnableComponent))
class CARRYGRAB_API UPlayerCarryComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UPlayerCarryComponent();
    const FCarryPlayerSettings* GetProfileSettings() const;
    float GetProbeFirstLength() const { return ProbeFirstLength; }
    float GetHandSweepRadius() const { return HandSweepRadius; }
    bool GetAccelerateRotation() const { return bAccelerateRotation; }
    float GetRotationStartSpeedDegrees() const { return RotationStartSpeedDegrees; }
    float GetRotationMaxSpeedDegrees() const { return RotationMaxSpeedDegrees; }
    float GetRotationRampTimeSeconds() const { return RotationRampTimeSeconds; }

    UFUNCTION(BlueprintCallable, Category = "Carry")
    void RequestGrab();

    UFUNCTION(BlueprintCallable, Category = "Carry")
    void RequestRelease();

    /** Start charging on press, throw on release, or clear the charge if the input was interrupted. */
    UFUNCTION(BlueprintCallable, Category = "Carry|Throw")
    void SetThrowChargeInput(bool bPressed, bool bCanceled);

    /** Move the held target one step out (positive) or in (negative). */
    UFUNCTION(BlueprintCallable, Category = "Carry")
    void AdjustHoldDistance(float Direction);

    /** Hold-to-rotate input, normalized to left (-1), released (0), or right (1). */
    UFUNCTION(BlueprintCallable, Category = "Carry")
    void SetCarryRotationInput(float Direction);

    UFUNCTION(BlueprintPure, Category = "Carry")
    bool IsHoldingItem() const;

    UFUNCTION(BlueprintPure, Category = "Carry")
    UCarryableComponent* GetHeldItem() const { return HeldItem.Get(); }

    /** Mass of the currently held item's root physics body in kilograms, or 0 if not holding. */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    float GetHeldItemMassKg() const;

    /** Latest raw grip force applied by this player on the held item, in newtons (replicated to owning client). */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    FVector GetGripForceNewtons() const { return CurrentGripForceNewtons; }

    /** Magnitude of the latest raw grip force applied by this player, in newtons. */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    float GetGripForceMagnitudeNewtons() const { return CurrentGripForceNewtons.Size(); }

    /** Smoothed grip force vector in newtons (filters out 1-frame solver spikes/drops for UI and movement). */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    FVector GetSmoothedGripForceNewtons() const { return SmoothedGripForceNewtons; }

    /** Smoothed grip force magnitude in newtons (recommended for driving CharacterMovement in Event Tick). */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    float GetSmoothedGripForceMagnitudeNewtons() const { return SmoothedGripForceNewtons.Size(); }

    /** Effective maximum pull force this player can apply, in newtons. */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    float GetMaxForceNewtons() const;

    /** Raw grip force magnitude divided by Max Force Newtons, clamped to 0..1. */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    float GetGripEffortRatio() const;

    /** Smoothed grip effort ratio (0..1, recommended for Lerp-ing MaxWalkSpeed/Acceleration/Braking in Event Tick). */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    float GetSmoothedGripEffortRatio() const;

    /** Effective carry settings, including the component's starting yaw speed in degrees per second. */
    UFUNCTION(BlueprintPure, Category = "Carry|Profile")
    FCarryPlayerSettings GetEffectivePlayerSettings() const;

    UPROPERTY(BlueprintAssignable, Category = "Carry")
    FCarryItemChangedSignature OnItemGrabbed;

    UPROPERTY(BlueprintAssignable, Category = "Carry")
    FCarryItemChangedSignature OnItemReleased;

    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    UFUNCTION(Server, Reliable)
    void ServerRequestGrab();

    UFUNCTION(Server, Reliable)
    void ServerRequestRelease();

    UFUNCTION(Server, Reliable)
    void ServerSetThrowChargeInput(bool bPressed, bool bCanceled);

    UFUNCTION(Server, Reliable)
    void ServerAdjustHoldDistance(int32 Direction);

    UFUNCTION(Server, Reliable)
    void ServerSetCarryRotationInput(int32 Direction);

    UFUNCTION()
    void HandleControllerChanged(APawn* Pawn, AController* OldController, AController* NewController);

    UFUNCTION()
    void OnRep_HeldItem(UCarryableComponent* PreviousItem);

    void ReleaseGrip();
    void RefreshGripForce(const APawn& Pawn, float DeltaTime);
    void UpdateSmoothedForce(float DeltaTime);

    /** Assign a Blueprint Player Carry Profile here. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Profile", meta = (AllowPrivateAccess = "true"))
    TObjectPtr<UPrimaryDataAsset> PlayerProfile;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Reach Trace", meta = (AllowPrivateAccess = "true", ClampMin = "1", Units = "cm", ToolTip = "Maximum length of trace 1 from elbow height to the bend. It shortens for nearby grips or to stay below an underside grip."))
    float ProbeFirstLength = 85.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Reach Trace", meta = (AllowPrivateAccess = "true", ClampMin = "1", Units = "cm", ToolTip = "Sphere radius used by both segments of the pickup trace."))
    float HandSweepRadius = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Reach Trace", meta = (AllowPrivateAccess = "true", ToolTip = "While carrying alone, move the grip to a visible point on the item when the center trace is blocked and suppress swing until the path clears."))
    bool bAutoRegrip = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Rotation", meta = (AllowPrivateAccess = "true", ToolTip = "When enabled, hold-to-rotate speeds up smoothly from Rotation Start Speed Degrees to Rotation Max Speed Degrees the longer the rotation input is held."))
    bool bAccelerateRotation = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Rotation", meta = (AllowPrivateAccess = "true", ClampMin = "0", ClampMax = "360", Units = "deg/s", ToolTip = "Starting yaw speed while holding a rotation button, in degrees per second. Also used as the constant speed when Accelerate Rotation is disabled."))
    float RotationStartSpeedDegrees = 70.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Rotation", meta = (AllowPrivateAccess = "true", EditCondition = "bAccelerateRotation", ClampMin = "0", ClampMax = "720", Units = "deg/s", ToolTip = "Maximum yaw speed reached after holding a rotation button for Rotation Ramp Time Seconds, in degrees per second."))
    float RotationMaxSpeedDegrees = 220.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Rotation", meta = (AllowPrivateAccess = "true", EditCondition = "bAccelerateRotation", ClampMin = "0.05", ClampMax = "10.0", Units = "s", ToolTip = "Seconds of continuous rotation input required to ramp from Rotation Start Speed Degrees to Rotation Max Speed Degrees."))
    float RotationRampTimeSeconds = 1.5f;

    UPROPERTY(EditAnywhere, Category = "Carry|Throw", meta = (ClampMin = "0.05", Units = "s", ToolTip = "How long to hold Throw to reach full power. Tapping uses 20 percent of the maximum impulse."))
    float ThrowChargeSeconds = 1.0f;

    UPROPERTY(EditAnywhere, Category = "Carry|Throw", meta = (ClampMin = "0", ToolTip = "Maximum impulse at full charge, in newton-seconds. Heavier items gain less speed from the same impulse."))
    float MaxThrowImpulse = 35.0f;

    UPROPERTY(EditAnywhere, Category = "Carry|Throw", meta = (ClampMin = "0", Units = "cm/s", ToolTip = "Maximum speed added by a full throw, in centimeters per second. Caps throws of very light props."))
    float MaxThrowAddedSpeed = 650.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Physics", meta = (AllowPrivateAccess = "true", ClampMin = "0.5", ClampMax = "60.0", ToolTip = "How quickly Smoothed Grip Force rises toward higher raw solver forces (1/s). Lower values filter out sudden wall-bump spikes."))
    float ForceRiseSmoothingRate = 6.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Carry|Physics", meta = (AllowPrivateAccess = "true", ClampMin = "0.5", ClampMax = "60.0", ToolTip = "How quickly Smoothed Grip Force decays toward lower forces (1/s). Lower values prevent sudden speed-up pulsing while walking."))
    float ForceFallSmoothingRate = 3.5f;

    /** Pickup reach when no player profile is assigned. */
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Carry", meta = (AllowPrivateAccess = "true", ClampMin = "1", Units = "cm", EditCondition = "PlayerProfile == nullptr", EditConditionHides))
    float MaxReach = 350.0f;

    /** Extra distance beyond pickup reach before a held grip starts its release countdown. */
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Carry", meta = (AllowPrivateAccess = "true", ClampMin = "0", Units = "cm", EditCondition = "PlayerProfile == nullptr", EditConditionHides))
    float HeldReachSlack = 60.0f;

    /** Continuous time outside held reach before this player's grip releases. */
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Carry", meta = (AllowPrivateAccess = "true", ClampMin = "0", Units = "s", EditCondition = "PlayerProfile == nullptr", EditConditionHides))
    float HeldReachGraceSeconds = 0.25f;

    UPROPERTY(ReplicatedUsing = OnRep_HeldItem, BlueprintReadOnly, Category = "Carry", meta = (AllowPrivateAccess = "true"))
    TObjectPtr<UCarryableComponent> HeldItem = nullptr;

    UPROPERTY(Replicated, BlueprintReadOnly, Category = "Carry|Physics", meta = (AllowPrivateAccess = "true"))
    FVector_NetQuantize10 CurrentGripForceNewtons = FVector::ZeroVector;

    UPROPERTY(Transient, BlueprintReadOnly, Category = "Carry|Physics", meta = (AllowPrivateAccess = "true"))
    FVector SmoothedGripForceNewtons = FVector::ZeroVector;

    double OutOfReachSince = -1.0;
    double ThrowChargeStarted = -1.0;
    int32 LocalRotationInput = 0;
};
