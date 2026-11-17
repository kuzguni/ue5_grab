#pragma once

#include "Components/ActorComponent.h"
#include "Physics/PhysicsInterfaceDeclares.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "CarryPhysicsComponent.generated.h"

class APawn;
class UCarryableComponent;
class UPrimitiveComponent;
struct FCarryGrip;
struct FCarryItemSettings;
struct FHitResult;

/** Drives each accepted holder's grip through a Chaos 6-DOF constraint on the authoritative rigid body. */
UCLASS(ClassGroup = (Carry), meta = (BlueprintSpawnableComponent))
class CARRYGRAB_API UCarryPhysicsComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UCarryPhysicsComponent();

    /** Immediately reconciles Chaos grip constraints and tick state with the owner's active grips. */
    void HandleGripsChanged();

    /** Latest force for one holder on the server, in newtons. */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    FVector GetGripForceNewtons(const APawn* Holder) const;

    /** Latest drive torque for one holder on the server, in newton-meters. */
    UFUNCTION(BlueprintPure, Category = "Carry|Physics")
    FVector GetGripTorqueNewtonMeters(const APawn* Holder) const;

protected:
    virtual void BeginPlay() override;
    virtual void OnUnregister() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    struct FActiveGripConstraint
    {
        FActiveGripConstraint()
            : PhysicsUserData(&ConstraintInstance)
        {
        }

        TWeakObjectPtr<const APawn> Holder = nullptr;
        FVector LocalPoint = FVector::ZeroVector;
        FTransform GrabbedLocalFrame = FTransform::Identity;
        FConstraintInstance ConstraintInstance;
        FPhysicsUserData PhysicsUserData;
        FPhysicsActorHandle KinematicHandle = nullptr;
        FPhysicsConstraintHandle ConstraintHandle;
    };

    FActiveGripConstraint* FindOrCreateGripConstraint(const FCarryGrip& Grip, const APawn& Holder,
        FPhysScene* Scene, const FPhysicsActorHandle& GrabbedHandle, const FTransform& GrabbedWorldTransform,
        const FVector& BodyContact, const FTransform& KinematicTarget);
    void PruneInactiveConstraints(const UCarryableComponent* Item);
    void ReleaseGripConstraint(FActiveGripConstraint& Entry);
    void ReleaseAllConstraints();

    struct FBodyDrive;
    struct FHolderDrive;

    FBodyDrive CalculateBodyDrive(const TArray<FCarryGrip>& Grips, int32 HolderCount,
        const FCarryItemSettings* Settings, double Mass, double GravityZ, double AvgInertia, double StepDt) const;
    static FHolderDrive CalculateHolderDrive(const FBodyDrive& Drive, bool bSolo, double ForceNewtons,
        double LiftForceNewtons, double TorqueNm);
    bool UpdateGripConstraintDrive(FActiveGripConstraint& ActiveConstraint, FCarryGrip& Grip, UPrimitiveComponent& Body,
        const UCarryableComponent& Item, const FBodyDrive& Drive, const FHolderDrive& HolderDrive,
        const FTransform& GrabbedWorldTransform, const FVector& BodyLocation, const FVector& BodyContact,
        const FTransform& KinematicTarget, double YawAlignVelZ);

    struct FHeavyContact
    {
        TWeakObjectPtr<UPrimitiveComponent> Body;
        FVector OutwardNormal;
        double OtherMass;
        double LastHitTime;
    };

    void TickReleasedBody(AActor& Owner, UPrimitiveComponent* Body, float DeltaTime);
    void RefreshHeavyObstacleTracking(const AActor& Owner, UPrimitiveComponent* Body, double Mass);
    void ClearHeavyObstacleTracking();
    void SetApproachTracking(UPrimitiveComponent* Body);
    void SetContactTracking(UPrimitiveComponent* Body);
    FVector LimitJointTarget(const FVector& Target, UPrimitiveComponent& Body, const UCarryableComponent& Item,
        const FVector& BodyLocation, const FVector& BodyContact, double MassPerHolder, double LinearStiffnessChaos);
    FVector LimitContactTarget(const FVector& Target, const FVector& GripPoint, double AllowedError) const;
    void UpdateHeavyApproach(UPrimitiveComponent& Body, const UCarryableComponent& Item,
        const FVector& DriveError, double Now);
    FVector LimitApproachTarget(const FVector& Target, const FVector& GripPoint,
        const FVector& BodyLocation, double AllowedError, double Now) const;

    UFUNCTION()
    void OnHeldBodyHit(UPrimitiveComponent* HitComponent, AActor* OtherActor,
        UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit);

    TWeakObjectPtr<UPrimitiveComponent> TrackedContactBody;
    TArray<FHeavyContact, TInlineAllocator<4>> HeavyContacts;
    TWeakObjectPtr<UPrimitiveComponent> TrackedApproachSourceBody;
    TWeakObjectPtr<UPrimitiveComponent> ApproachBody;
    FVector ApproachNormal = FVector::ZeroVector;
    FVector ApproachStopOffset = FVector::ZeroVector;
    double ApproachMass = 0.0;
    double LastApproachTime = 0.0;
    double NextApproachQuery = 0.0;
    double TrackedBodyMass = 0.0;
    bool bEnabledHitNotifications = false;

    double PostReleaseReplicationTimer = 0.0;

    /** Spring strength in newtons per meter. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Items Without Profiles", meta = (ClampMin = "1.0", ToolTip = "Spring strength in newtons per meter. Only used when Carryable has no Item Carry Profile."))
    float Stiffness = 2500.0f;

    /** Target follow rate for props at or below the reference mass per holder, in inverse seconds. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Items Without Profiles", meta = (ClampMin = "1.0", ToolTip = "Target follow rate for props at or below the reference mass per holder, in inverse seconds. Only used when Carryable has no Item Carry Profile."))
    float TargetFollowRate = 18.0f;

    /** Props above this mass per holder ease into movement more slowly. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Items Without Profiles", meta = (ClampMin = "0.1", Units = "kg", ToolTip = "Props above this mass per holder ease into movement more slowly. Only used when Carryable has no Item Carry Profile."))
    float ReferenceFollowMassKg = 5.0f;

    /** Minimum fraction of the light-prop follow rate available to heavy props. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Items Without Profiles", meta = (ClampMin = "0.1", ClampMax = "1.0", ToolTip = "Minimum fraction of the light-prop follow rate available to heavy props. Only used when Carryable has no Item Carry Profile."))
    float MinimumHeavyFollowScale = 0.45f;

    /** Maximum force a single holder can apply, in newtons. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Players Without Profiles", meta = (ClampMin = "0.0", ToolTip = "Maximum force a single holder can apply, in newtons. Only used for a holder without a Player Carry Profile."))
    float MaxHolderForceNewtons = 700.0f;

    /** Upper bound on total force per kilogram, for stable handling of lighter objects. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Items Without Profiles", meta = (ClampMin = "0.0", Units = "m/s^2", ToolTip = "Upper bound on total force per kilogram, for stable handling of lighter objects. Only used when Carryable has no Item Carry Profile."))
    float MaxCarryAcceleration = 40.0f;

    /** Angular velocity damping, in inverse seconds, when holders keep their aim steady. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Items Without Profiles", meta = (ClampMin = "0.0", ToolTip = "Angular velocity damping, in inverse seconds, when holders keep their aim steady. Only used when Carryable has no Item Carry Profile."))
    float AngularSettleRate = 7.5f;

    /** Maximum stabilizing torque per holder, in newton meters. */
    UPROPERTY(EditDefaultsOnly, AdvancedDisplay, Category = "Carry|Fallbacks|Players Without Profiles", meta = (ClampMin = "0.0", ToolTip = "Maximum stabilizing torque per holder, in newton meters. Only used for a holder without a Player Carry Profile."))
    float MaxStabilizingTorqueNm = 80.0f;

    TArray<TUniquePtr<FActiveGripConstraint>> ActiveConstraints;

    UPROPERTY(Transient)
    TObjectPtr<UCarryableComponent> CachedCarryable = nullptr;

    UPROPERTY(Transient)
    TObjectPtr<UPrimitiveComponent> CachedBody = nullptr;
};
