#pragma once

#include "CoreMinimal.h"
#include "CarryProfileSettings.generated.h"

class UPhysicalMaterial;

USTRUCT(BlueprintType)
struct CARRYGRAB_API FCarryPlayerSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Reach", meta = (ClampMin = "1", Units = "cm", ToolTip = "Maximum distance from the camera to a surface when starting a grab, in centimeters."))
    float PickupReach = 350.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Reach", meta = (ClampMin = "0", Units = "cm", ToolTip = "Extra distance allowed beyond Pickup Reach while holding. Measured from the pawn eye position to its actual grab point, in centimeters."))
    float HeldReachSlack = 60.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Reach", meta = (ClampMin = "0", Units = "s", ToolTip = "Seconds the grab point must stay outside held reach before this player lets go. Returning to range resets the countdown."))
    float ReleaseGrace = 0.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hand Reach Preview", meta = (ClampMin = "1", Units = "cm", ToolTip = "Maximum reach from the elbow-height origin for pickup. The separate held-distance grace still governs release."))
    float HandReachDistance = 180.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hand Reach Preview|Bent Probe", meta = (Units = "cm", ToolTip = "Vertical offset of the first sweep's start from the capsule center. Use 0 to start at the center."))
    float ElbowHeightOffset = 35.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hand Reach Preview|Bent Probe", meta = (ClampMin = "0", ClampMax = "89", ToolTip = "The first sweep's upward angle from horizontal. The second sweep aims at the selected grab point."))
    float ProbeRiseDegrees = 70.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls", meta = (ClampMin = "1", Units = "cm", ToolTip = "Distance the hold target moves per mouse wheel notch."))
    float HoldDistanceStep = 15.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls", meta = (ClampMin = "1", ToolTip = "How quickly the scroll target eases toward the selected distance, in inverse seconds. Higher is faster. The item profile slows this rate for heavier loads per holder."))
    float HoldDistanceFollowRate = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls", meta = (ClampMin = "1", Units = "cm", ToolTip = "Closest the scroll control normally moves the target. Picking up closer is still allowed."))
    float MinimumHoldDistance = 55.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controls", meta = (ClampMin = "0", ClampMax = "120", ToolTip = "Legacy profile field. Player carry components use their own Rotation Start Speed Degrees. Effective Player Settings reports the component's starting speed."))
    float RotationSpeedDegrees = 70.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Effort", meta = (ClampMin = "0", ToolTip = "Maximum pull this player can supply to one grip, in newtons. Other holders have their own limits."))
    float MaxForceNewtons = 700.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Effort", meta = (ClampMin = "0", ToolTip = "Maximum vertical grip force this player can supply, in newtons. The general pull limit also applies. Holders contribute independently."))
    float MaxLiftForceNewtons = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Effort", meta = (ClampMin = "0", ToolTip = "Maximum twist this player can supply to stabilize one grip, in newton-meters. Set to zero to disable this player's angular drive."))
    float MaxTorqueNewtonMeters = 80.0f;
};

USTRUCT(BlueprintType)
struct CARRYGRAB_API FCarryItemSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (ToolTip = "Use Mass Kg when play begins. Leave off to keep the mass configured on the item's mesh."))
    bool bOverrideMass = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (EditCondition = "bOverrideMass", ClampMin = "0.01", Units = "kg", ToolTip = "Simulated body mass in kilograms. Higher values take more force to lift and accelerate. Requires Override Mass."))
    float MassKg = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (ToolTip = "Use the body damping values below when play begins. Leave off to keep the mesh settings."))
    bool bOverrideBodyDamping = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (EditCondition = "bOverrideBodyDamping", ClampMin = "0", ToolTip = "Resistance to the body's linear motion, including when dropped. Higher values slow translation more quickly. Requires Override Body Damping."))
    float BodyLinearDamping = 0.01f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (EditCondition = "bOverrideBodyDamping", ClampMin = "0", ToolTip = "Resistance to the body's spin, including when dropped. Higher values make rotation settle sooner. Requires Override Body Damping."))
    float BodyAngularDamping = 0.0f;

    /** Leave empty to use the mesh physical material. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (ToolTip = "Optional override for the item's physical material, which controls effects such as friction and bounce. Leave empty to use the mesh material."))
    TObjectPtr<UPhysicalMaterial> PhysicalMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (ClampMin = "1", ClampMax = "4", ToolTip = "Maximum players allowed to hold this item at once. A player contributes one grip."))
    int32 MaxHolders = 4;

    /** Grip spring stiffness in N/m. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive", meta = (ClampMin = "1", ToolTip = "Grip spring stiffness in newtons per meter. Higher values pull harder for the same distance from the target, up to each player's force limit."))
    float Stiffness = 2500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive", meta = (ClampMin = "0", ToolTip = "Scales both linear and angular grip damping. Higher values resist oscillation more while the item is held."))
    float DampingRatio = 0.9f;

    /** Limit on the acceleration the grips can request together, in m/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive", meta = (ClampMin = "0", ToolTip = "Maximum acceleration the grips can request together, in meters per second squared. Player force limits can reduce it further."))
    float MaxAcceleration = 40.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Follow", meta = (ClampMin = "1", ToolTip = "How quickly the grip target follows camera aim, in inverse seconds. Higher values reduce target lag."))
    float TargetFollowRate = 18.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Follow", meta = (ClampMin = "0.1", Units = "kg", ToolTip = "Mass per holder where heavier loads start to follow more slowly. Also sets the reference point for turn and swing tuning."))
    float ReferenceFollowMassKg = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Follow", meta = (ClampMin = "0.1", ClampMax = "1", ToolTip = "Lowest fraction of Target Follow Rate allowed as the load per holder increases. For example, 0.45 allows the rate to fall to 45 percent."))
    float MinimumHeavyFollowScale = 0.45f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0", ClampMax = "1", ToolTip = "Fraction of available grip acceleration used to sweep the hold target during camera yaw and pitch. Higher values let the target move faster."))
    float TurnAccelerationFraction = 0.27f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0", ClampMax = "1", ToolTip = "Multiplier on target turn acceleration at the reference load and above. Lower values ease the pull on heavier items during fast camera turns."))
    float LoadedTurnAccelerationScale = 0.60f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0.01", Units = "s", ToolTip = "Controls how early the target eases off its turn speed as it catches up. Higher values start braking sooner. Measured in seconds."))
    float TurnBrakingTime = 0.38f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0", ToolTip = "Base rate at which the grip damps the prop's spin, in inverse seconds. Higher values reduce continued rotation after a turn."))
    float AngularSettleRate = 7.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0", ToolTip = "Angular damping multiplier for the lightest loads during fast turns. Lower values allow more swing."))
    float LightFastTurnDampingScale = 0.55f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0", ToolTip = "Angular damping multiplier at the Reference Follow Mass per holder. Higher values reduce swing near that load."))
    float ReferenceAngularDampingScale = 1.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "1", ClampMax = "5", ToolTip = "Extra held angular damping for compact props. The effect fades out as the body's rotational size increases. Set to 1 to disable it."))
    float CompactSwingDampingScale = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Swing", meta = (ClampMin = "0", ToolTip = "Angular damping multiplier for very heavy loads, reached at seven times the Reference Follow Mass per holder."))
    float HeavyAngularDampingScale = 1.5f;
};
