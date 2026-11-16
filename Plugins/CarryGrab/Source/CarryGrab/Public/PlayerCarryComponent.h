#pragma once

#include "Components/ActorComponent.h"
#include "PlayerCarryComponent.generated.h"

class AController;
class APawn;
class UCarryableComponent;
class UPrimaryDataAsset;
struct FCarryPlayerSettings;

UCLASS(ClassGroup = (Carry), meta = (BlueprintSpawnableComponent))
class CARRYGRAB_API UPlayerCarryComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UPlayerCarryComponent();
    const FCarryPlayerSettings* GetProfileSettings() const;
    float GetProbeFirstLength() const { return ProbeFirstLength; }
    float GetHandSweepRadius() const { return HandSweepRadius; }

    UFUNCTION(BlueprintCallable, Category = "Carry")
    void RequestGrab();

    UFUNCTION(BlueprintCallable, Category = "Carry")
    void RequestRelease();

    /** Move the held target one step out (positive) or in (negative). */
    UFUNCTION(BlueprintCallable, Category = "Carry")
    void AdjustHoldDistance(float Direction);

    /** Hold-to-rotate input, normalized to left (-1), released (0), or right (1). */
    UFUNCTION(BlueprintCallable, Category = "Carry")
    void SetCarryRotationInput(float Direction);

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
    void ServerAdjustHoldDistance(int32 Direction);

    UFUNCTION(Server, Reliable)
    void ServerSetCarryRotationInput(int32 Direction);

    UFUNCTION()
    void HandleControllerChanged(APawn* Pawn, AController* OldController, AController* NewController);

    void ReleaseGrip();

    /** Assign a Blueprint Player Carry Profile here. */
    UPROPERTY(EditAnywhere, Category = "Carry|Profile")
    TObjectPtr<UPrimaryDataAsset> PlayerProfile;

    UPROPERTY(EditAnywhere, Category = "Carry|Reach Trace", meta = (ClampMin = "1", Units = "cm", ToolTip = "Maximum length of trace 1 from elbow height to the bend. It shortens for nearby grips or to stay below an underside grip."))
    float ProbeFirstLength = 85.0f;

    UPROPERTY(EditAnywhere, Category = "Carry|Reach Trace", meta = (ClampMin = "1", Units = "cm", ToolTip = "Sphere radius used by both segments of the pickup trace."))
    float HandSweepRadius = 2.0f;

    UPROPERTY(EditAnywhere, Category = "Carry|Reach Trace", meta = (ToolTip = "While carrying alone, move the grip to a visible point on the item when the center trace is blocked and suppress swing until the path clears."))
    bool bAutoRegrip = true;

    /** Pickup reach when no player profile is assigned. */
    UPROPERTY(EditDefaultsOnly, Category = "Carry", meta = (ClampMin = "1", Units = "cm", EditCondition = "PlayerProfile == nullptr", EditConditionHides))
    float MaxReach = 350.0f;

    /** Extra distance beyond pickup reach before a held grip starts its release countdown. */
    UPROPERTY(EditDefaultsOnly, Category = "Carry", meta = (ClampMin = "0", Units = "cm", EditCondition = "PlayerProfile == nullptr", EditConditionHides))
    float HeldReachSlack = 60.0f;

    /** Continuous time outside held reach before this player's grip releases. */
    UPROPERTY(EditDefaultsOnly, Category = "Carry", meta = (ClampMin = "0", Units = "s", EditCondition = "PlayerProfile == nullptr", EditConditionHides))
    float HeldReachGraceSeconds = 0.25f;

    TWeakObjectPtr<UCarryableComponent> HeldItem;
    double OutOfReachSince = -1.0;
    int32 LocalRotationInput = 0;
};
