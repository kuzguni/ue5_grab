#include "CarryPhysicsComponent.h"

#include "Data/CarryReplicationRates.h"
#include "Carry/CarryConstants.h"
#include "Carry/CarryDrive.h"
#include "CarryProfileSettings.h"
#include "PlayerCarryComponent.h"

#include "CarryableComponent.h"
#include "World/CarryView.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"

namespace
{
    constexpr float ActiveReplicationThresholdHz = 10.5f;
    constexpr double PostReleaseSettleWindowSeconds = 2.0;
    constexpr double EarlySettleRemainingSeconds = 1.85;
    constexpr double SettledLinearSpeedSquared = 25.0;
    constexpr double SettledAngularSpeedSquared = 0.01;
}

UCarryPhysicsComponent::UCarryPhysicsComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UCarryPhysicsComponent::BeginPlay()
{
    Super::BeginPlay();
    if (const AActor* Owner = GetOwner())
    {
        CachedCarryable = Owner->FindComponentByClass<UCarryableComponent>();
        CachedBody = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    }
}

void UCarryPhysicsComponent::OnUnregister()
{
    ClearHeavyObstacleTracking();
    ReleaseAllConstraints();
    Super::OnUnregister();
}

void UCarryPhysicsComponent::HandleGripsChanged()
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        ClearHeavyObstacleTracking();
        ReleaseAllConstraints();
        PostReleaseReplicationTimer = 0.0;
        SetComponentTickEnabled(false);
        return;
    }

    UCarryableComponent* Item = CachedCarryable ? CachedCarryable.Get() : Owner->FindComponentByClass<UCarryableComponent>();
    UPrimitiveComponent* Body = CachedBody ? CachedBody.Get() : Cast<UPrimitiveComponent>(Owner->GetRootComponent());

    if (!Owner->HasAuthority())
    {
        ClearHeavyObstacleTracking();
        ReleaseAllConstraints();
        PostReleaseReplicationTimer = 0.0;
        Owner->SetMinNetUpdateFrequency(Carry::SettledNetUpdateFrequencyHz);
        SetComponentTickEnabled(Item && !Item->Grips.IsEmpty());
        return;
    }

    if (!Item || Item->Grips.IsEmpty())
    {
        ClearHeavyObstacleTracking();
        const bool bHadConstraints = !ActiveConstraints.IsEmpty();
        ReleaseAllConstraints();
        if (Body)
        {
            Body->WakeAllRigidBodies();
        }

        if ((bHadConstraints || Owner->GetMinNetUpdateFrequency() > ActiveReplicationThresholdHz) && Body && Body->IsSimulatingPhysics())
        {
            PostReleaseReplicationTimer = PostReleaseSettleWindowSeconds;
            Owner->SetMinNetUpdateFrequency(Carry::HeldNetUpdateFrequencyHz);
            SetComponentTickEnabled(true);
        }
        else
        {
            PostReleaseReplicationTimer = 0.0;
            Owner->SetMinNetUpdateFrequency(Carry::SettledNetUpdateFrequencyHz);
            SetComponentTickEnabled(false);
        }
        return;
    }

    PostReleaseReplicationTimer = 0.0;
    Owner->SetMinNetUpdateFrequency(Carry::HeldNetUpdateFrequencyHz);
    PruneInactiveConstraints(Item);
    if (Body)
    {
        Body->WakeAllRigidBodies();
    }
    SetComponentTickEnabled(true);
}

void UCarryPhysicsComponent::TickReleasedBody(AActor& Owner, UPrimitiveComponent* Body, float DeltaTime)
{
    ClearHeavyObstacleTracking();
    ReleaseAllConstraints();
    if (Owner.HasAuthority() && PostReleaseReplicationTimer > 0.0 && Body && Body->IsSimulatingPhysics())
    {
        PostReleaseReplicationTimer -= DeltaTime;
        const double LinearVelSq = Body->GetPhysicsLinearVelocity().SizeSquared();
        const double AngularVelSq = Body->GetPhysicsAngularVelocityInRadians().SizeSquared();
        if (PostReleaseReplicationTimer <= 0.0 || (PostReleaseReplicationTimer < EarlySettleRemainingSeconds
            && LinearVelSq < SettledLinearSpeedSquared && AngularVelSq < SettledAngularSpeedSquared))
        {
            PostReleaseReplicationTimer = 0.0;
            Owner.SetMinNetUpdateFrequency(Carry::SettledNetUpdateFrequencyHz);
            SetComponentTickEnabled(false);
        }
        return;
    }

    PostReleaseReplicationTimer = 0.0;
    Owner.SetMinNetUpdateFrequency(Carry::SettledNetUpdateFrequencyHz);
    SetComponentTickEnabled(false);
}

FVector UCarryPhysicsComponent::GetGripForceNewtons(const APawn* Holder) const
{
    const UCarryableComponent* Item = CachedCarryable ? CachedCarryable.Get() : (GetOwner() ? GetOwner()->FindComponentByClass<UCarryableComponent>() : nullptr);
    if (Item && Holder)
    {
        const FCarryGrip* Grip = Item->Grips.FindByPredicate([Holder](const FCarryGrip& Entry) { return Entry.Holder == Holder; });
        if (Grip)
        {
            return Grip->AppliedForce / Carry::CentimetersPerMeter;
        }
    }
    return FVector::ZeroVector;
}

void UCarryPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    AActor* Owner = GetOwner();
    if (!Owner)
    {
        ClearHeavyObstacleTracking();
        ReleaseAllConstraints();
        PostReleaseReplicationTimer = 0.0;
        SetComponentTickEnabled(false);
        return;
    }

    UCarryableComponent* Item = CachedCarryable ? CachedCarryable.Get() : Owner->FindComponentByClass<UCarryableComponent>();
    UPrimitiveComponent* Body = CachedBody ? CachedBody.Get() : Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    if (!Item || Item->Grips.IsEmpty())
    {
        TickReleasedBody(*Owner, Body, DeltaTime);
        return;
    }

    if (DeltaTime <= UE_SMALL_NUMBER)
    {
        return;
    }

    const bool bHasAuthority = Owner->HasAuthority();
    if (bHasAuthority)
    {
        const bool bHasInvalidGrip = Item->Grips.ContainsByPredicate([](const FCarryGrip& Grip) { return !IsValid(Grip.Holder.Get()); });
        if (bHasInvalidGrip)
        {
            Item->RemoveInvalidGrips();
            if (Item->Grips.IsEmpty())
            {
                HandleGripsChanged();
                return;
            }
        }
    }
    else
    {
        ReleaseAllConstraints();
    }

    FBodyInstance* BodyInstance = Body ? Body->GetBodyInstance() : nullptr;
    if (BodyInstance && BodyInstance->WeldParent != nullptr)
    {
        BodyInstance = BodyInstance->WeldParent;
    }
    const FPhysicsActorHandle GrabbedHandle = BodyInstance ? BodyInstance->GetPhysicsActor() : nullptr;
    if (!BodyInstance || !Body->IsSimulatingPhysics() || !FPhysicsInterface::IsValid(GrabbedHandle))
    {
        ClearHeavyObstacleTracking();
        ReleaseAllConstraints();
        for (FCarryGrip& Grip : Item->Grips)
        {
            Grip.AppliedForce = FVector::ZeroVector;
        }
        return;
    }

    int32 ValidHolderCount = 0;
    for (const FCarryGrip& Grip : Item->Grips)
    {
        if (IsValid(Grip.Holder.Get()))
        {
            ++ValidHolderCount;
        }
    }
    if (ValidHolderCount == 0)
    {
        ClearHeavyObstacleTracking();
        ReleaseAllConstraints();
        for (FCarryGrip& Grip : Item->Grips)
        {
            Grip.AppliedForce = FVector::ZeroVector;
        }
        return;
    }

    if (bHasAuthority)
    {
        PruneInactiveConstraints(Item);
    }

    const double StepDt = FMath::Max(static_cast<double>(DeltaTime), UE_SMALL_NUMBER);
    const double Mass = FMath::Max(static_cast<double>(Body->GetMass()), UE_SMALL_NUMBER);
    if (bHasAuthority)
    {
        RefreshHeavyObstacleTracking(*Owner, Body, Mass);
    }
    const FVector Inertia = BodyInstance->GetBodyInertiaTensor();
    const double AvgInertia = FMath::Max((Inertia.X + Inertia.Y + Inertia.Z) / 3.0, 1.0);
    const FTransform BodyTransform = Body->GetComponentTransform();
    FTransform GrabbedWorldTransform(BodyTransform.GetRotation(), BodyTransform.GetLocation());
    FPhysicsCommand::ExecuteRead(GrabbedHandle, [&](const FPhysicsActorHandle& Actor)
    {
        GrabbedWorldTransform = FPhysicsInterface::GetGlobalPose_AssumesLocked(Actor);
    });

    const int32 HolderCount = ValidHolderCount;
    const FCarryItemSettings* Settings = Item->GetProfileSettings();
    const double GravityZ = Owner->GetWorld() ? Owner->GetWorld()->GetGravityZ() : -980.0;
    const FBodyDrive Drive = CalculateBodyDrive(Item->Grips, HolderCount, Settings, Mass, GravityZ, AvgInertia, StepDt);

    FPhysScene* Scene = BodyInstance->GetPhysicsScene();
    int32 ActiveHolders = 0;

    for (FCarryGrip& Grip : Item->Grips)
    {
        const APawn* Holder = Grip.Holder.Get();
        if (!IsValid(Holder))
        {
            Grip.AppliedForce = FVector::ZeroVector;
            continue;
        }

        ++ActiveHolders;

        const UPlayerCarryComponent* PlayerCarry = Holder->FindComponentByClass<UPlayerCarryComponent>();
        const FCarryPlayerSettings* PlayerSettings = PlayerCarry ? PlayerCarry->GetProfileSettings() : nullptr;
        const double ForceNewtons = PlayerSettings ? PlayerSettings->MaxForceNewtons : MaxHolderForceNewtons;
        const double TorqueNm = PlayerSettings ? PlayerSettings->MaxTorqueNewtonMeters : MaxStabilizingTorqueNm;
        const FHolderDrive HolderDrive = CalculateHolderDrive(Drive, HolderCount == 1, ForceNewtons, TorqueNm);

        const Carry::FGripTarget Target = Carry::UpdateGripTarget(Grip, *Holder, GetCarryViewTransform(*Holder), GrabbedWorldTransform.GetRotation(),
            PlayerSettings ? PlayerSettings->HoldDistanceFollowRate : 10.0f, Drive.Follow, HolderDrive.TurningAcceleration);
        const double YawAlignVelZ = Carry::UpdateGripYaw(Grip, Target, Drive.Follow, HolderCount == 1,
            GrabbedWorldTransform.GetRotation(), HolderDrive.AngularOrientScale, PlayerCarry, PlayerSettings);

        const FVector BodyContact = BodyTransform.TransformPosition(Grip.LocalPoint);
        const FQuat TargetYawQuat = FRotator(0.0, Target.YawDegrees, 0.0).Quaternion();
        const FTransform KinematicTarget(TargetYawQuat, Grip.SmoothedTarget);

        if (bHasAuthority && Scene)
        {
            FActiveGripConstraint* ActiveConstraint = FindOrCreateGripConstraint(Grip, *Holder, Scene,
                GrabbedHandle, GrabbedWorldTransform, BodyContact, KinematicTarget);
            if (ActiveConstraint && ActiveConstraint->KinematicHandle && ActiveConstraint->ConstraintHandle.IsValid())
            {
                if (UpdateGripConstraintDrive(*ActiveConstraint, Grip, *Body, *Item, Drive, HolderDrive,
                    GrabbedWorldTransform, BodyTransform.GetLocation(), BodyContact, KinematicTarget, YawAlignVelZ))
                {
                    continue;
                }
            }
        }
        else if (!bHasAuthority && !HolderDrive.PerHolderGravityHold.IsNearlyZero())
        {
            Body->AddForce(HolderDrive.PerHolderGravityHold);
        }

        // Initial grab frame before solver scatter (or non-authority client display): analytical spring-damper estimate.
        const FVector Error = (Grip.SmoothedTarget - BodyContact).GetClampedToMaxSize(HolderDrive.FullForceExtension);
        const FVector PointVelocity = Body->GetPhysicsLinearVelocityAtPoint(BodyContact);
        const FVector ForceEstimate = Drive.LinearStiffnessChaos * Error + Drive.LinearDampingChaos * (Target.Velocity - PointVelocity);
        Grip.AppliedForce = ForceEstimate.GetClampedToMaxSize(HolderDrive.MaxForce);
    }

    if (bHasAuthority && ActiveHolders > 0)
    {
        Body->WakeRigidBody();
    }
}
