#include "CarryPhysicsComponent.h"

#include "CarryProfileSettings.h"
#include "PlayerCarryComponent.h"

#include "CarryableComponent.h"
#include "CarryView.h"
#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/BodyInstance.h"
#include "ProfilingDebugging/CsvProfiler.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/PBDJointConstraintData.h"
#include "Chaos/ShapeInstance.h"
#include "Chaos/Sphere.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

CSV_DEFINE_CATEGORY(CarryContacts, false);

namespace
{
    constexpr double CentimetersPerMeter = 100.0;
    constexpr double DampingRatio = 0.9;
    constexpr double LookSpeedDecayRate = 12.0;
    constexpr double SwingStartSpeed = 60.0; // cm/s of target motion from turning the view
    constexpr double DeliberateSwingSpeed = 260.0;
    constexpr double TurnAccelerationFraction = 0.27;
    constexpr double MinTurnRadius = 8.0; // cm, below this yaw has little physical effect
    constexpr double TurnBrakingTime = 0.38; // seconds from full sideways speed to rest
    constexpr double MovingAngularDampingScale = 0.55;
    constexpr double ReferenceAngularDampingScale = 1.25;
    constexpr double HeavyAngularDampingScale = 1.5;
    constexpr double ContactExpirySeconds = 0.09;
    constexpr double MinMassRatio = 2.0;
    constexpr double FullLimitMassRatio = 4.0;
    constexpr double HeavyContactAcceleration = 4.0; // m/s² of additional lateral spring force into contact
    constexpr int32 MaxHeavyContacts = 4;
    constexpr double ApproachQueryInterval = 0.05;
    constexpr double ApproachExpiry = 0.12;
    constexpr double ApproachDistance = 90.0;
    constexpr double ApproachDriveThreshold = 16.0;
    constexpr double ApproachErrorFraction = 0.15;

    TAutoConsoleVariable<int32> CVarLimitHeavyContactPush(
        TEXT("carry.LimitHeavyContactPush"), 1,
        TEXT("Limit a held prop's sideways drive into much heavier movable carryables. 0: off, 1: on."));
    TAutoConsoleVariable<int32> CVarBrakeHeavyApproach(
        TEXT("carry.BrakeHeavyApproach"), 1,
        TEXT("Brake fast held props approaching a much heavier movable carryable. 0: off, 1: on."));
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
    SetContactTracking(nullptr);
    ReleaseAllConstraints();
    Super::OnUnregister();
}

void UCarryPhysicsComponent::SetContactTracking(UPrimitiveComponent* Body)
{
    if (TrackedContactBody.Get() == Body && !TrackedContactBody.IsStale())
    {
        return;
    }

    if (UPrimitiveComponent* Previous = TrackedContactBody.Get())
    {
        Previous->OnComponentHit.RemoveDynamic(this, &UCarryPhysicsComponent::OnHeldBodyHit);
        if (bEnabledHitNotifications)
        {
            Previous->SetNotifyRigidBodyCollision(false);
        }
    }
    TrackedContactBody = Body;
    HeavyContacts.Reset();
    ApproachBody.Reset();
    NextApproachQuery = 0.0;
    bEnabledHitNotifications = false;
    if (Body)
    {
        const FBodyInstance* Instance = Body->GetBodyInstance();
        bEnabledHitNotifications = Instance && !Instance->bNotifyRigidBodyCollision;
        Body->OnComponentHit.AddUniqueDynamic(this, &UCarryPhysicsComponent::OnHeldBodyHit);
        if (bEnabledHitNotifications)
        {
            Body->SetNotifyRigidBodyCollision(true);
        }
    }
}

void UCarryPhysicsComponent::OnHeldBodyHit(UPrimitiveComponent* HitComponent, AActor* OtherActor,
    UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit)
{
    CSV_SCOPED_TIMING_STAT(CarryContacts, OnHeldBodyHit);
    const FVector Normal = Hit.ImpactNormal.GetSafeNormal2D();
    if (HitComponent != TrackedContactBody.Get() || !IsValid(OtherActor) || !IsValid(OtherComponent)
        || OtherActor == GetOwner() || FMath::Abs(Hit.ImpactNormal.Z) > 0.5 || Normal.IsNearlyZero()
        || !OtherComponent->IsSimulatingPhysics(Hit.BoneName))
    {
        return;
    }

    const double Now = GetWorld()->GetTimeSeconds();
    FHeavyContact* Match = HeavyContacts.FindByPredicate([&](const FHeavyContact& Contact)
    {
        return Contact.Body == OtherComponent && FVector::DotProduct(Contact.OutwardNormal, Normal) > 0.95;
    });
    if (Match)
    {
        Match->OutwardNormal = Normal;
        Match->LastHitTime = Now;
        return;
    }

    const double OtherMass = OtherComponent->GetMass();
    if (OtherMass <= TrackedBodyMass * MinMassRatio || !OtherActor->FindComponentByClass<UCarryableComponent>())
    {
        return;
    }

    if (HeavyContacts.Num() < MaxHeavyContacts)
    {
        HeavyContacts.Add({ OtherComponent, Normal, OtherMass, Now });
        return;
    }
    FHeavyContact* Oldest = &HeavyContacts[0];
    for (FHeavyContact& Contact : HeavyContacts)
    {
        if (Contact.LastHitTime < Oldest->LastHitTime)
        {
            Oldest = &Contact;
        }
    }
    *Oldest = { OtherComponent, Normal, OtherMass, Now };
}

FVector UCarryPhysicsComponent::LimitContactTarget(const FVector& Target, const FVector& GripPoint, double AllowedError) const
{
    CSV_SCOPED_TIMING_STAT(CarryContacts, LimitContactTarget);
    FVector Error = Target - GripPoint;
    for (const FHeavyContact& Contact : HeavyContacts)
    {
        const double InwardError = -FVector::DotProduct(Error, Contact.OutwardNormal);
        if (InwardError > AllowedError)
        {
            const double Limit = FMath::SmoothStep(MinMassRatio, FullLimitMassRatio, Contact.OtherMass / TrackedBodyMass);
            Error += Contact.OutwardNormal * (InwardError - AllowedError) * Limit;
        }
    }
    return GripPoint + Error;
}

void UCarryPhysicsComponent::UpdateHeavyApproach(UPrimitiveComponent& Body, const UCarryableComponent& Item,
    const FVector& DriveError, double Now)
{
    if (Now < NextApproachQuery || DriveError.SizeSquared2D() < FMath::Square(ApproachDriveThreshold))
    {
        return;
    }
    NextApproachQuery = Now + ApproachQueryInterval;
    CSV_SCOPED_TIMING_STAT(CarryContacts, SweepHeavyApproach);

    FComponentQueryParams Query(SCENE_QUERY_STAT(CarryHeavyApproach), GetOwner());
    Query.bFindInitialOverlaps = false;
    for (const FCarryGrip& Grip : Item.Grips)
    {
        Query.AddIgnoredActor(Grip.Holder.Get());
    }
    const FVector Start = Body.GetComponentLocation();
    const FVector Direction = DriveError.GetSafeNormal2D();
    TArray<FHitResult> Hits;
    GetWorld()->ComponentSweepMulti(Hits, &Body, Start, Start + Direction * ApproachDistance, Body.GetComponentQuat(), Query);
    for (const FHitResult& Hit : Hits)
    {
        UPrimitiveComponent* Other = Hit.GetComponent();
        const FVector Normal = Hit.ImpactNormal.GetSafeNormal2D();
        if (!Hit.bBlockingHit || Hit.bStartPenetrating || FMath::Abs(Hit.ImpactNormal.Z) > 0.5
            || FVector::DotProduct(Normal, Direction) >= -0.2 || !IsValid(Other) || !Other->IsSimulatingPhysics(Hit.BoneName))
        {
            continue;
        }

        const double OtherMass = Other->GetMass();
        if (OtherMass <= TrackedBodyMass * MinMassRatio || !Other->GetOwner()->FindComponentByClass<UCarryableComponent>())
        {
            continue;
        }
        ApproachBody = Other;
        ApproachNormal = Normal;
        ApproachStopOffset = Hit.Location - Other->GetComponentLocation();
        ApproachMass = OtherMass;
        LastApproachTime = Now;
        break;
    }
}

FVector UCarryPhysicsComponent::LimitApproachTarget(const FVector& Target, const FVector& GripPoint,
    const FVector& BodyLocation, double AllowedError, double Now) const
{
    const UPrimitiveComponent* Other = ApproachBody.Get();
    if (!Other || Now - LastApproachTime > ApproachExpiry)
    {
        return Target;
    }
    const double Gap = FVector::DotProduct(BodyLocation - Other->GetComponentLocation() - ApproachStopOffset, ApproachNormal);
    const double InwardError = -FVector::DotProduct(Target - GripPoint, ApproachNormal);
    const double SafeError = FMath::Max(AllowedError, ApproachErrorFraction * Gap);
    if (InwardError <= SafeError)
    {
        return Target;
    }
    const double Limit = FMath::SmoothStep(MinMassRatio, FullLimitMassRatio, ApproachMass / TrackedBodyMass);
    return Target + ApproachNormal * (InwardError - SafeError) * Limit;
}

void UCarryPhysicsComponent::ReleaseGripConstraint(FActiveGripConstraint& Entry)
{
    if (Entry.ConstraintHandle.IsValid())
    {
        FPhysicsInterface::ReleaseConstraint(Entry.ConstraintHandle);
    }
    if (Entry.KinematicHandle)
    {
        FChaosScene* Scene = FChaosEngineInterface::GetCurrentScene(Entry.KinematicHandle);
        FChaosEngineInterface::ReleaseActor(Entry.KinematicHandle, Scene);
    }
    Entry.ConstraintInstance.Reset();
    Entry.Holder = nullptr;
}

void UCarryPhysicsComponent::ReleaseAllConstraints()
{
    if (ActiveConstraints.IsEmpty())
    {
        return;
    }

    for (TUniquePtr<FActiveGripConstraint>& Entry : ActiveConstraints)
    {
        if (Entry)
        {
            ReleaseGripConstraint(*Entry);
        }
    }
    ActiveConstraints.Reset();
}

void UCarryPhysicsComponent::PruneInactiveConstraints(const UCarryableComponent* Item)
{
    for (int32 Index = ActiveConstraints.Num() - 1; Index >= 0; --Index)
    {
        const TUniquePtr<FActiveGripConstraint>& Entry = ActiveConstraints[Index];
        const bool bStillHeld = Entry
            && IsValid(Entry->Holder.Get())
            && Item
            && Item->Grips.ContainsByPredicate([&Entry](const FCarryGrip& Grip)
            {
                return Grip.Holder == Entry->Holder && Grip.LocalPoint.Equals(Entry->LocalPoint, 0.5);
            });
        if (!bStillHeld)
        {
            if (Entry)
            {
                ReleaseGripConstraint(*Entry);
            }
            ActiveConstraints.RemoveAtSwap(Index, EAllowShrinking::No);
        }
    }
}

void UCarryPhysicsComponent::HandleGripsChanged()
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        SetContactTracking(nullptr);
        ReleaseAllConstraints();
        PostReleaseReplicationTimer = 0.0;
        SetComponentTickEnabled(false);
        return;
    }

    UCarryableComponent* Item = CachedCarryable ? CachedCarryable.Get() : Owner->FindComponentByClass<UCarryableComponent>();
    UPrimitiveComponent* Body = CachedBody ? CachedBody.Get() : Cast<UPrimitiveComponent>(Owner->GetRootComponent());

    if (!Owner->HasAuthority())
    {
        SetContactTracking(nullptr);
        ReleaseAllConstraints();
        PostReleaseReplicationTimer = 0.0;
        Owner->SetMinNetUpdateFrequency(10.0f);
        SetComponentTickEnabled(Item && !Item->Grips.IsEmpty());
        return;
    }

    if (!Item || Item->Grips.IsEmpty())
    {
        SetContactTracking(nullptr);
        const bool bHadConstraints = !ActiveConstraints.IsEmpty();
        ReleaseAllConstraints();
        if (Body)
        {
            Body->WakeAllRigidBodies();
        }

        if ((bHadConstraints || Owner->GetMinNetUpdateFrequency() > 10.5f) && Body && Body->IsSimulatingPhysics())
        {
            PostReleaseReplicationTimer = 2.0;
            Owner->SetMinNetUpdateFrequency(60.0f);
            SetComponentTickEnabled(true);
        }
        else
        {
            PostReleaseReplicationTimer = 0.0;
            Owner->SetMinNetUpdateFrequency(10.0f);
            SetComponentTickEnabled(false);
        }
        return;
    }

    PostReleaseReplicationTimer = 0.0;
    Owner->SetMinNetUpdateFrequency(60.0f);
    PruneInactiveConstraints(Item);
    if (Body)
    {
        Body->WakeAllRigidBodies();
    }
    SetComponentTickEnabled(true);
}

FVector UCarryPhysicsComponent::GetGripForceNewtons(const APawn* Holder) const
{
    const UCarryableComponent* Item = CachedCarryable ? CachedCarryable.Get() : (GetOwner() ? GetOwner()->FindComponentByClass<UCarryableComponent>() : nullptr);
    if (Item && Holder)
    {
        const FCarryGrip* Grip = Item->Grips.FindByPredicate([Holder](const FCarryGrip& Entry) { return Entry.Holder == Holder; });
        if (Grip)
        {
            return Grip->AppliedForce / CentimetersPerMeter;
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
        SetContactTracking(nullptr);
        ReleaseAllConstraints();
        PostReleaseReplicationTimer = 0.0;
        SetComponentTickEnabled(false);
        return;
    }

    UCarryableComponent* Item = CachedCarryable ? CachedCarryable.Get() : Owner->FindComponentByClass<UCarryableComponent>();
    UPrimitiveComponent* Body = CachedBody ? CachedBody.Get() : Cast<UPrimitiveComponent>(Owner->GetRootComponent());
    if (!Item || Item->Grips.IsEmpty())
    {
        SetContactTracking(nullptr);
        ReleaseAllConstraints();
        if (Owner->HasAuthority() && PostReleaseReplicationTimer > 0.0 && Body && Body->IsSimulatingPhysics())
        {
            PostReleaseReplicationTimer -= DeltaTime;
            const double LinearVelSq = Body->GetPhysicsLinearVelocity().SizeSquared();
            const double AngularVelSq = Body->GetPhysicsAngularVelocityInRadians().SizeSquared();
            if (PostReleaseReplicationTimer <= 0.0 || (PostReleaseReplicationTimer < 1.85 && LinearVelSq < 25.0 && AngularVelSq < 0.01))
            {
                PostReleaseReplicationTimer = 0.0;
                Owner->SetMinNetUpdateFrequency(10.0f);
                SetComponentTickEnabled(false);
            }
            return;
        }

        PostReleaseReplicationTimer = 0.0;
        Owner->SetMinNetUpdateFrequency(10.0f);
        SetComponentTickEnabled(false);
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
        SetContactTracking(nullptr);
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
        SetContactTracking(nullptr);
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
        TrackedBodyMass = Mass;
        SetContactTracking(CVarLimitHeavyContactPush.GetValueOnGameThread() != 0 ? Body : nullptr);
        if (!HeavyContacts.IsEmpty())
        {
            const double Cutoff = Owner->GetWorld()->GetTimeSeconds() - ContactExpirySeconds;
            HeavyContacts.RemoveAllSwap([Cutoff](const FHeavyContact& Contact)
            {
                return Contact.LastHitTime < Cutoff || !Contact.Body.IsValid();
            }, EAllowShrinking::No);
        }
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
    const double MassPerHolder = Mass / HolderCount;
    const double GravityZ = Owner->GetWorld() ? Owner->GetWorld()->GetGravityZ() : -980.0;
    const double DriveDampingRatio = Settings ? Settings->DampingRatio : DampingRatio;
    const double LinearStiffnessChaos = FMath::Max(1.0, static_cast<double>(Settings ? Settings->Stiffness : Stiffness));
    const double LinearDampingChaos = 2.0 * DriveDampingRatio * FMath::Sqrt(LinearStiffnessChaos * MassPerHolder);
    const double ReferenceMass = FMath::Max(0.1, static_cast<double>(Settings ? Settings->ReferenceFollowMassKg : ReferenceFollowMassKg));
    const double AccelerationCap = Settings ? Settings->MaxAcceleration : MaxCarryAcceleration;
    const double BrakingTime = Settings ? FMath::Max(0.01, static_cast<double>(Settings->TurnBrakingTime)) : TurnBrakingTime;

    const double TurnLoadAlpha = FMath::SmoothStep(
        0.4 * ReferenceMass, ReferenceMass, MassPerHolder);
    const double TurnFraction = Settings ? Settings->TurnAccelerationFraction : TurnAccelerationFraction;
    const double LoadedTurnScale = Settings ? Settings->LoadedTurnAccelerationScale : 0.60;
    const double FollowScale = FMath::Clamp(FMath::Sqrt(ReferenceMass / MassPerHolder),
        static_cast<double>(Settings ? Settings->MinimumHeavyFollowScale : MinimumHeavyFollowScale), 1.0);
    const double FollowRate = Settings ? Settings->TargetFollowRate : TargetFollowRate;
    const double TargetAlpha = 1.0 - FMath::Exp(-FollowRate * FollowScale * StepDt);
    const double LookSpeedDecay = FMath::Exp(-LookSpeedDecayRate * StepDt);

    const double GyrationRadiusCm = FMath::Sqrt(AvgInertia / Mass);

    double FastestLookSpeed = 0.0;
    for (const FCarryGrip& Grip : Item->Grips)
    {
        FastestLookSpeed = FMath::Max(FastestLookSpeed, Grip.RecentLookSpeed);
    }
    const double SwingAlpha = FMath::Clamp((FastestLookSpeed - SwingStartSpeed) /
        (DeliberateSwingSpeed - SwingStartSpeed), 0.0, 1.0);
    const double LightDamping = Settings ? Settings->LightFastTurnDampingScale : MovingAngularDampingScale;
    const double ReferenceDamping = Settings ? Settings->ReferenceAngularDampingScale : ReferenceAngularDampingScale;
    const double HeavyDamping = Settings ? Settings->HeavyAngularDampingScale : HeavyAngularDampingScale;
    const double DampingSwingScale = FMath::Lerp(1.0, LightDamping, SwingAlpha);
    // Keep the fast-turn damping reduction on the lightest props only.
    const double ReferenceDampingScale = FMath::Lerp(DampingSwingScale, ReferenceDamping, TurnLoadAlpha);
    const double HeavyLoadAlpha = FMath::SmoothStep(ReferenceMass, 7.0 * ReferenceMass, MassPerHolder);
    const double AngularDampingScale = FMath::Lerp(ReferenceDampingScale, HeavyDamping, HeavyLoadAlpha);
    const double SettleRate = Settings ? Settings->AngularSettleRate : AngularSettleRate;
    const double AngularDampingChaos = DriveDampingRatio * SettleRate * AngularDampingScale * AvgInertia;

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
        const double MaxForce = CentimetersPerMeter * FMath::Max(0.0, FMath::Min(ForceNewtons, MassPerHolder * AccelerationCap));
        const double GravitySupportForce = FMath::Clamp(-MassPerHolder * GravityZ, 0.0, MaxForce);
        const FVector PerHolderGravityHold(0.0, 0.0, GravitySupportForce);
        const double FullForceExtension = MaxForce / LinearStiffnessChaos;
        const double TurningAcceleration = TurnFraction * MaxForce / MassPerHolder * FMath::Lerp(1.0, LoadedTurnScale, TurnLoadAlpha);
        const double WeightRatio = (-MassPerHolder * GravityZ) / FMath::Max(MaxForce, 1.0);
        const double AngularOrientScale = HolderCount == 1 ? 1.0 - FMath::SmoothStep(0.85, 1.15, WeightRatio) : 0.0;
        const double AngularTorqueCapChaos = FMath::Min(FMath::Max(0.0, TorqueNm) * FMath::Square(CentimetersPerMeter), 0.5 * MaxForce * GyrationRadiusCm);
        const double RemainingHorizSq = FMath::Max(0.0, FMath::Square(MaxForce) - FMath::Square(GravitySupportForce));
        const double PerHorizCap = FMath::Max(0.45 * MaxForce, FMath::Sqrt(RemainingHorizSq)) / UE_SQRT_2;

        const FTransform RawView = GetCarryViewTransform(*Holder);
        const FVector AimDirection = RawView.TransformVectorNoScale(
            Grip.ViewLocalTarget.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector));
        const double DesiredHoldDistance = Grip.ViewLocalTarget.Size();
        if (Grip.InitializedHolder != Holder || !Grip.InitializedLocalPoint.Equals(Grip.LocalPoint, 0.5))
        {
            Grip.InitializedHolder = Holder;
            Grip.InitializedLocalPoint = Grip.LocalPoint;
            const FVector InitialOffset = AimDirection * DesiredHoldDistance;
            Grip.SmoothedTarget = RawView.GetLocation() + InitialOffset;
            Grip.PreviousAimDirection = AimDirection;
            Grip.SmoothedHoldDistance = DesiredHoldDistance;
            Grip.CarryTargetYaw = InitialOffset.Size2D() >= 1.0
                ? FMath::Atan2(InitialOffset.Y, InitialOffset.X)
                : FMath::DegreesToRadians(RawView.GetRotation().Rotator().Yaw);
            Grip.SmoothedHorizontalReach = InitialOffset.Size2D();
            Grip.LastRawViewLocation = RawView.GetLocation();
            Grip.LastRawViewRotation = RawView.GetRotation();
            Grip.RemoteViewAngularVelocity = FVector::ZeroVector;
            Grip.TimeSinceViewSample = 0.0;
            Grip.bStabilizeTurnDistance = false;
            Grip.bEaseOutFastTurn = false;
            Grip.RecentLookSpeed = 0.0;
            Grip.InitialBodyRotation = GrabbedWorldTransform.GetRotation();
            Grip.InitialViewYaw = FMath::RadiansToDegrees(Grip.CarryTargetYaw);
        }

        const double LocDeltaSq = FVector::DistSquared(RawView.GetLocation(), Grip.LastRawViewLocation);
        const double RotDelta = Grip.LastRawViewRotation.AngularDistance(RawView.GetRotation());
        const bool bRawViewChanged = Holder->IsLocallyControlled() || LocDeltaSq > 1e-4 || RotDelta > 1e-5;

        const FTransform View = RawView;
        const double DistanceRate = FMath::Max(1.0, static_cast<double>(PlayerSettings ? PlayerSettings->HoldDistanceFollowRate : 10.0f));
        const double DistanceAlpha = 1.0 - FMath::Exp(-DistanceRate * FollowScale * StepDt);
        Grip.SmoothedHoldDistance = FMath::Lerp(Grip.SmoothedHoldDistance, DesiredHoldDistance, DistanceAlpha);
        const FVector ViewOffset = AimDirection * Grip.SmoothedHoldDistance;
        bool bHoldFastTurnState = false;
        if (bRawViewChanged)
        {
            const double SampleDt = Holder->IsLocallyControlled()
                ? StepDt
                : FMath::Clamp(Grip.TimeSinceViewSample + StepDt, StepDt, 0.1);
            Grip.LastRawViewLocation = RawView.GetLocation();
            Grip.LastRawViewRotation = RawView.GetRotation();
            Grip.TimeSinceViewSample = 0.0;
            const double LookSpeed = (AimDirection - Grip.PreviousAimDirection).Size() * Grip.SmoothedHoldDistance / SampleDt;
            Grip.PreviousAimDirection = AimDirection;
            Grip.RecentLookSpeed = FMath::Max(LookSpeed, Grip.RecentLookSpeed * LookSpeedDecay);
        }
        else
        {
            Grip.TimeSinceViewSample += StepDt;
            Grip.RecentLookSpeed *= LookSpeedDecay;
            bHoldFastTurnState = Grip.TimeSinceViewSample <= 0.025;
        }

        const FVector PreviousTarget = Grip.SmoothedTarget;
        FVector TargetOffset = ViewOffset;
        const double TurnRadius = ViewOffset.Size2D();
        const double DesiredYaw = TurnRadius >= 1.0
            ? FMath::Atan2(ViewOffset.Y, ViewOffset.X)
            : FMath::DegreesToRadians(RawView.GetRotation().Rotator().Yaw);
        if (TurnRadius >= MinTurnRadius)
        {
            // v^2 / r is the inward acceleration needed to keep following this arc.
            // Limit how fast the target sweeps sideways so the grip has force left to pull inward.
            const double MaxYawSpeed = FMath::Sqrt(TurningAcceleration / TurnRadius);
            const double YawError = FMath::FindDeltaAngleRadians(Grip.CarryTargetYaw, DesiredYaw);
            const double YawDistance = FMath::Abs(YawError);
            const bool bReachedTurnLimit = YawDistance > MaxYawSpeed * StepDt;
            Grip.bStabilizeTurnDistance |= bReachedTurnLimit;
            Grip.bEaseOutFastTurn |= bReachedTurnLimit;

            // Start braking while there is still an arc left to travel. Using v^2 = 2*a*d
            // ramps down the target's sideways speed instead of stopping it in one tick.
            const double ArrivalSpeed = Grip.bEaseOutFastTurn
                ? FMath::Sqrt(2.0 * (MaxYawSpeed / BrakingTime) * YawDistance)
                : MaxYawSpeed;
            const double StepLimit = FMath::Min(MaxYawSpeed, ArrivalSpeed) * StepDt;
            // Let faster turns move the target faster even after it crosses the
            // base speed limit. Bound the extra speed so the grip can catch up.
            const double ExcessStep = FMath::Max(0.0, YawDistance - StepLimit);
            const double ExtraSpeedBudget = 0.5 * StepLimit;
            const double ExtraStep = ExcessStep * ExtraSpeedBudget
                / FMath::Max(ExcessStep + ExtraSpeedBudget, UE_SMALL_NUMBER);
            const double YawStep = FMath::Sign(YawError) * (FMath::Min(YawDistance, StepLimit) + ExtraStep);
            if (!bHoldFastTurnState)
            {
                Grip.bEaseOutFastTurn &= !FMath::IsNearlyEqual(YawStep, YawError);
            }
            Grip.CarryTargetYaw = FMath::UnwindRadians(Grip.CarryTargetYaw + YawStep);
            TargetOffset.X = TurnRadius * FMath::Cos(Grip.CarryTargetYaw);
            TargetOffset.Y = TurnRadius * FMath::Sin(Grip.CarryTargetYaw);
        }
        else
        {
            Grip.CarryTargetYaw = DesiredYaw;
            Grip.bEaseOutFastTurn = false;
        }

        const FVector Target = View.GetLocation() + TargetOffset;
        Grip.SmoothedTarget = FMath::Lerp(PreviousTarget, Target, TargetAlpha);
        Grip.SmoothedHorizontalReach = FMath::Lerp(Grip.SmoothedHorizontalReach, TurnRadius, TargetAlpha);

        const FVector TargetVelocity = (Grip.SmoothedTarget - PreviousTarget) / StepDt;
        const double HorizontalMotionSpeed = FMath::Max(TargetVelocity.Size2D(), Grip.RecentLookSpeed);
        const double MotionAlpha = FMath::SmoothStep(10.0, 80.0, HorizontalMotionSpeed);
        const double CurrentTargetYawDeg = FMath::RadiansToDegrees(Grip.CarryTargetYaw);

        if (HolderCount != 1)
        {
            Grip.RotationInput = 0;
            Grip.bHasManualYawTarget = false;
            Grip.bSuppressSwing = false;
            Grip.bRegrippedWhileBlocked = false;
            Grip.ClearReachSince = -1.0;
            Grip.SwingHoldAlpha = 0.0;
        }
        if (Grip.RotationInput != 0 && !Grip.bHasManualYawTarget)
        {
            Grip.bHasManualYawTarget = true;
            Grip.ManualYawTarget = GrabbedWorldTransform.GetRotation();
        }

        double YawAlignVelZ = 0.0;
        if (Grip.bHasManualYawTarget)
        {
            const double RotationSpeed = FMath::Clamp(static_cast<double>(PlayerSettings ? PlayerSettings->RotationSpeedDegrees : 70.0f), 0.0, 120.0);
            const double YawStep = FMath::DegreesToRadians(RotationSpeed) * FollowScale * Grip.RotationInput * StepDt;
            Grip.ManualYawTarget = FQuat(FVector::UpVector, YawStep) * Grip.ManualYawTarget;
            const FQuat Error = Grip.ManualYawTarget * GrabbedWorldTransform.GetRotation().Inverse();
            FQuat Swing, Twist;
            Error.ToSwingTwist(FVector::UpVector, Swing, Twist);
            const double YawError = FMath::UnwindRadians(2.0 * FMath::Atan2(Twist.Z, Twist.W));
            const double BoundedError = FMath::Clamp(YawError, -UE_PI / 5.0, UE_PI / 5.0);
            // Avoid storing up a large rotation behind an obstacle while the button is held.
            Grip.ManualYawTarget = FQuat(FVector::UpVector, BoundedError - YawError) * Grip.ManualYawTarget;
            Grip.ManualYawTarget.Normalize();
            YawAlignVelZ = AngularOrientScale * 4.0 * BoundedError;
        }
        else if (MotionAlpha <= UE_KINDA_SMALL_NUMBER || AngularOrientScale <= UE_KINDA_SMALL_NUMBER)
        {
            Grip.InitialBodyRotation = GrabbedWorldTransform.GetRotation();
            Grip.InitialViewYaw = CurrentTargetYawDeg;
        }
        else
        {
            const double DeltaViewYawDeg = FMath::FindDeltaAngleDegrees(Grip.InitialViewYaw, CurrentTargetYawDeg);
            const FQuat YawCarriedRefQuat = FRotator(0.0, DeltaViewYawDeg, 0.0).Quaternion() * Grip.InitialBodyRotation;
            const FQuat ToTargetQuat = YawCarriedRefQuat * GrabbedWorldTransform.GetRotation().Inverse();
            FQuat WorldSwingQuat = FQuat::Identity;
            FQuat WorldYawTwistQuat = FQuat::Identity;
            ToTargetQuat.ToSwingTwist(FVector::UpVector, WorldSwingQuat, WorldYawTwistQuat);
            if (WorldYawTwistQuat.W < 0.0)
            {
                WorldYawTwistQuat = -WorldYawTwistQuat;
            }

            const double YawErrorRad = 2.0 * FMath::Atan2(WorldYawTwistQuat.Z, WorldYawTwistQuat.W);
            constexpr double MaxTrackedYawLagRad = UE_PI / 5.0; // 36 deg
            const double ClampedYawErrorRad = FMath::Clamp(YawErrorRad, -MaxTrackedYawLagRad, MaxTrackedYawLagRad);
            if (!FMath::IsNearlyEqual(YawErrorRad, ClampedYawErrorRad, 1e-4))
            {
                const FQuat ExcessTwistQuat(FVector::UpVector, ClampedYawErrorRad - YawErrorRad);
                Grip.InitialBodyRotation = ExcessTwistQuat * Grip.InitialBodyRotation;
            }

            const double SwingAngleRad = WorldSwingQuat.GetAngle();
            const double SwingValidity = FMath::SmoothStep(2.2, 1.4, SwingAngleRad);
            constexpr double YawCatchupGain = 3.0;
            constexpr double MaxYawCatchupSpeed = 2.5;
            YawAlignVelZ = AngularOrientScale * MotionAlpha * SwingValidity
                * FMath::Clamp(ClampedYawErrorRad * YawCatchupGain, -MaxYawCatchupSpeed, MaxYawCatchupSpeed);
        }

        const FVector BodyContact = BodyTransform.TransformPosition(Grip.LocalPoint);
        const FQuat TargetYawQuat = FRotator(0.0, CurrentTargetYawDeg, 0.0).Quaternion();
        const FTransform KinematicTarget(TargetYawQuat, Grip.SmoothedTarget);

        if (bHasAuthority && Scene)
        {
            FActiveGripConstraint* ActiveConstraint = nullptr;
            for (const TUniquePtr<FActiveGripConstraint>& Existing : ActiveConstraints)
            {
                if (Existing && Existing->Holder == Holder && Existing->LocalPoint.Equals(Grip.LocalPoint, 0.5))
                {
                    ActiveConstraint = Existing.Get();
                    break;
                }
            }

            if (!ActiveConstraint)
            {
                TUniquePtr<FActiveGripConstraint> NewEntry = MakeUnique<FActiveGripConstraint>();
                NewEntry->Holder = Holder;
                NewEntry->LocalPoint = Grip.LocalPoint;

                const FTransform InitialTargetPose(TargetYawQuat, BodyContact);
                NewEntry->GrabbedLocalFrame = InitialTargetPose.GetRelativeTransform(GrabbedWorldTransform);

                FActorCreationParams Params;
                Params.Scene = Scene;
                Params.bSimulatePhysics = false;
                Params.bQueryOnly = false;
                Params.bStatic = false;
                Params.InitialTM = KinematicTarget;
                FPhysicsInterface::CreateActor(Params, NewEntry->KinematicHandle);

                if (NewEntry->KinematicHandle)
                {
                    auto Sphere = MakeImplicitObjectPtr<Chaos::FImplicitSphere3>(FVector(0.0, 0.0, 0.0), 0.0f);
                    NewEntry->KinematicHandle->GetGameThreadAPI().SetGeometry(Sphere);
                    NewEntry->KinematicHandle->GetGameThreadAPI().SetUserData(nullptr);
                    NewEntry->KinematicHandle->GetGameThreadAPI().SetObjectState(Chaos::EObjectStateType::Kinematic);
                    for (const TUniquePtr<Chaos::FPerShapeData>& Shape : NewEntry->KinematicHandle->GetGameThreadAPI().ShapesArray())
                    {
                        Chaos::FCollisionData CollisionData = Shape->GetCollisionData();
                        CollisionData.bQueryCollision = false;
                        CollisionData.bSimCollision = false;
                        Shape->SetCollisionData(CollisionData);
                    }

                    FPhysicsInterface::AddActorToSolver(NewEntry->KinematicHandle, Scene->GetSolver());
                    NewEntry->ConstraintInstance.PhysScene = Scene;
                    NewEntry->KinematicHandle->GetGameThreadAPI().SetKinematicTarget(KinematicTarget);

                    NewEntry->ConstraintHandle = FChaosEngineInterface::CreateConstraint(
                        NewEntry->KinematicHandle,
                        GrabbedHandle,
                        FTransform::Identity,
                        NewEntry->GrabbedLocalFrame);

                    if (NewEntry->ConstraintHandle.IsValid() && NewEntry->ConstraintHandle.Constraint->IsType(Chaos::EConstraintType::JointConstraintType))
                    {
                        if (Chaos::FJointConstraint* Constraint = static_cast<Chaos::FJointConstraint*>(NewEntry->ConstraintHandle.Constraint))
                        {
                            Constraint->SetUserData(&NewEntry->PhysicsUserData);
                            NewEntry->ConstraintInstance.ConstraintHandle = NewEntry->ConstraintHandle;

                            Constraint->SetCollisionEnabled(false);
                            Constraint->SetProjectionEnabled(false);
                            Constraint->SetUseLinearSolver(true);

                            Constraint->SetLinearMotionTypesX(Chaos::EJointMotionType::Free);
                            Constraint->SetLinearMotionTypesY(Chaos::EJointMotionType::Free);
                            Constraint->SetLinearMotionTypesZ(Chaos::EJointMotionType::Free);
                            Constraint->SetAngularMotionTypesX(Chaos::EJointMotionType::Free);
                            Constraint->SetAngularMotionTypesY(Chaos::EJointMotionType::Free);
                            Constraint->SetAngularMotionTypesZ(Chaos::EJointMotionType::Free);

                            Constraint->SetLinearPositionDriveEnabled(Chaos::TVec3<bool>(true));
                            Constraint->SetLinearVelocityDriveEnabled(Chaos::TVec3<bool>(true));
                            Constraint->SetLinearDrivePositionTarget(FVector::ZeroVector);
                            Constraint->SetLinearDriveVelocityTarget(FVector::ZeroVector);
                            Constraint->SetLinearDriveForceMode(Chaos::EJointForceMode::Force);

                            Constraint->SetAngularSLerpPositionDriveEnabled(false);
                            Constraint->SetAngularSLerpVelocityDriveEnabled(true);
                            Constraint->SetAngularDrivePositionTarget(FQuat::Identity);
                            Constraint->SetAngularDriveVelocityTarget(FVector::ZeroVector);
                            Constraint->SetAngularDriveForceMode(Chaos::EJointForceMode::Force);
                        }

                        ActiveConstraint = NewEntry.Get();
                        ActiveConstraints.Add(MoveTemp(NewEntry));
                    }
                    else
                    {
                        ReleaseGripConstraint(*NewEntry);
                    }
                }
            }

            if (ActiveConstraint && ActiveConstraint->KinematicHandle && ActiveConstraint->ConstraintHandle.IsValid())
            {
                const FQuat ConnectorWorldRot = GrabbedWorldTransform.GetRotation() * ActiveConstraint->GrabbedLocalFrame.GetRotation();
                const FMatrix ConnectorAxesMatrix = ConnectorWorldRot.ToMatrix();
                FVector AxisMaxForce(PerHorizCap, PerHorizCap, MaxForce);
                double SumVertCap = 0.0;
                for (int32 AxisIdx = 0; AxisIdx < 3; ++AxisIdx)
                {
                    const FVector AxisW = ConnectorAxesMatrix.GetScaledAxis(static_cast<EAxis::Type>(EAxis::X + AxisIdx));
                    const double HorizSq = AxisW.X * AxisW.X + AxisW.Y * AxisW.Y;
                    const double VertSq = AxisW.Z * AxisW.Z;
                    const double InvCapSq = HorizSq / FMath::Square(FMath::Max(PerHorizCap, 1.0))
                        + VertSq / FMath::Square(FMath::Max(MaxForce, 1.0));
                    const double Cap = InvCapSq > UE_SMALL_NUMBER ? 1.0 / FMath::Sqrt(InvCapSq) : MaxForce;
                    AxisMaxForce[AxisIdx] = Cap;
                    SumVertCap += FMath::Abs(AxisW.Z) * Cap;
                }
                if (SumVertCap > MaxForce && SumVertCap > UE_SMALL_NUMBER)
                {
                    AxisMaxForce *= (MaxForce / SumVertCap);
                }

                if (Grip.RegripTime >= 0.0)
                {
                    constexpr double RegripBlendTime = 0.12;
                    AxisMaxForce *= FMath::SmoothStep(0.0, RegripBlendTime,
                        Owner->GetWorld()->GetTimeSeconds() - Grip.RegripTime + StepDt);
                }

                // Ramp the temporary pose drive in and out. The regular angular velocity drive still controls yaw.
                Grip.SwingHoldAlpha = FMath::FInterpConstantTo(Grip.SwingHoldAlpha, Grip.bSuppressSwing ? 1.0 : 0.0,
                    StepDt, Grip.bSuppressSwing ? 12.0 : 5.0);
                const double HoldTorque = Grip.SwingHoldAlpha * AngularTorqueCapChaos * AngularOrientScale;
                const bool bHoldTilt = HoldTorque > UE_SMALL_NUMBER;
                FTransform JointTarget = KinematicTarget;
                if (TrackedContactBody.IsValid())
                {
                    // Keep the camera target intact. Limit only the endpoint that drives the
                    // joint, and only along the horizontal normal toward a heavy contact.
                    const double AllowedError = HeavyContactAcceleration * CentimetersPerMeter * MassPerHolder / LinearStiffnessChaos;
                    if (!HeavyContacts.IsEmpty())
                    {
                        JointTarget.SetLocation(LimitContactTarget(JointTarget.GetLocation(), BodyContact, AllowedError));
                    }
                    else if (CVarBrakeHeavyApproach.GetValueOnGameThread() != 0)
                    {
                        const double Now = Owner->GetWorld()->GetTimeSeconds();
                        UpdateHeavyApproach(*Body, *Item, KinematicTarget.GetLocation() - BodyContact, Now);
                        JointTarget.SetLocation(LimitApproachTarget(JointTarget.GetLocation(), BodyContact,
                            BodyTransform.GetLocation(), AllowedError, Now));
                    }
                }
                if (bHoldTilt)
                {
                    const FQuat RotationFromHold = GrabbedWorldTransform.GetRotation() * Grip.SwingHoldRotation.Inverse();
                    FQuat Tilt, Yaw;
                    RotationFromHold.ToSwingTwist(FVector::UpVector, Tilt, Yaw);
                    JointTarget.SetRotation(Yaw * Grip.SwingHoldRotation * ActiveConstraint->GrabbedLocalFrame.GetRotation());
                }
                const double PoseStiffness = HoldTorque / FMath::DegreesToRadians(5.0);
                const double PoseDamping = 2.0 * DriveDampingRatio * FMath::Sqrt(PoseStiffness * AvgInertia);
                const double AngularDamping = FMath::Max(AngularDampingChaos, PoseDamping);
                const FVector LocalAngVelTarget = ConnectorWorldRot.UnrotateVector(FVector(0.0, 0.0, -YawAlignVelZ));

                FPhysicsCommand::ExecuteWrite(ActiveConstraint->ConstraintHandle, [&](const FPhysicsConstraintHandle& InConstraint)
                {
                    FPhysicsInterface::SetKinematicTarget_AssumesLocked(ActiveConstraint->KinematicHandle, JointTarget);
                    if (Chaos::FJointConstraint* Constraint = static_cast<Chaos::FJointConstraint*>(InConstraint.Constraint))
                    {
                        Constraint->SetLinearPositionDriveEnabled(Chaos::TVec3<bool>(MaxForce > 0.0));
                        Constraint->SetLinearVelocityDriveEnabled(Chaos::TVec3<bool>(MaxForce > 0.0));
                        Constraint->SetLinearDriveStiffness(Chaos::FVec3(LinearStiffnessChaos));
                        Constraint->SetLinearDriveDamping(Chaos::FVec3(LinearDampingChaos));
                        Constraint->SetLinearDriveMaxForce(Chaos::FVec3(AxisMaxForce.X, AxisMaxForce.Y, AxisMaxForce.Z));

                        Constraint->SetAngularSLerpPositionDriveEnabled(bHoldTilt);
                        Constraint->SetAngularSLerpVelocityDriveEnabled(AngularTorqueCapChaos > 0.0);
                        Constraint->SetAngularDriveVelocityTarget(Chaos::FVec3(LocalAngVelTarget.X, LocalAngVelTarget.Y, LocalAngVelTarget.Z));
                        Constraint->SetAngularDriveStiffness(Chaos::FVec3(PoseStiffness));
                        Constraint->SetAngularDriveDamping(Chaos::FVec3(AngularDamping));
                        Constraint->SetAngularDriveMaxTorque(Chaos::FVec3(AngularTorqueCapChaos));
                    }
                });

                FVector RawLinImpulse = FVector::ZeroVector;
                FVector RawAngImpulse = FVector::ZeroVector;
                FPhysicsInterface::GetForce(ActiveConstraint->ConstraintHandle, RawLinImpulse, RawAngImpulse);
                const FVector SolverForce = -RawLinImpulse / StepDt;
                if (SolverForce.SizeSquared() > 1.0)
                {
                    Grip.AppliedForce = SolverForce.GetClampedToMaxSize(MaxForce);
                    continue;
                }
            }
        }
        else if (!bHasAuthority && !PerHolderGravityHold.IsNearlyZero())
        {
            Body->AddForce(PerHolderGravityHold);
        }

        // Initial grab frame before solver scatter (or non-authority client display): analytical spring-damper estimate.
        const FVector Error = (Grip.SmoothedTarget - BodyContact).GetClampedToMaxSize(FullForceExtension);
        const FVector PointVelocity = Body->GetPhysicsLinearVelocityAtPoint(BodyContact);
        const FVector ForceEstimate = LinearStiffnessChaos * Error + LinearDampingChaos * (TargetVelocity - PointVelocity);
        Grip.AppliedForce = ForceEstimate.GetClampedToMaxSize(MaxForce);
    }

    if (bHasAuthority && ActiveHolders > 0)
    {
        Body->WakeRigidBody();
    }
}
