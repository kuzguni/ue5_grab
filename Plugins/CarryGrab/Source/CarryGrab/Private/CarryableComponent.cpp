#include "CarryableComponent.h"

#include "CarryCollisionComponent.h"
#include "CarryPhysicsComponent.h"
#include "CarryProfileBinding.h"
#include "CarryProfileSettings.h"
#include "CarryView.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"
#include "Components/CapsuleComponent.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerState.h"
#include "HAL/IConsoleManager.h"
#include "Net/UnrealNetwork.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PlayerCarryComponent.h"

namespace
{
    struct FGripReach
    {
        FVector Start = FVector::ZeroVector;
        FVector Bend = FVector::ZeroVector;
        FVector End = FVector::ZeroVector;
        float Radius = 0.0f;
        int32 BlockedLeg = 0;

        bool CanReach() const { return Radius > 0.0f && BlockedLeg == 0; }
    };

    FGripReach EvaluateGripReach(UWorld& World, const APawn& Holder, const UPrimitiveComponent& Body,
        const FVector& Contact, const FVector& Normal)
    {
        static const FCarryPlayerSettings Defaults;
        const UPlayerCarryComponent* PlayerCarry = Holder.FindComponentByClass<UPlayerCarryComponent>();
        const FCarryPlayerSettings* Profile = PlayerCarry ? PlayerCarry->GetProfileSettings() : nullptr;
        const FCarryPlayerSettings& Settings = Profile ? *Profile : Defaults;
        const UPlayerCarryComponent* ProbeSettings = PlayerCarry ? PlayerCarry : GetDefault<UPlayerCarryComponent>();

        const UCapsuleComponent* Capsule = Cast<UCapsuleComponent>(Holder.GetRootComponent());
        const FVector Center = Capsule ? Capsule->GetComponentLocation() : Holder.GetActorLocation();
        FGripReach Reach;
        Reach.Start = Center + FVector::UpVector * Settings.ElbowHeightOffset;
        const FVector ToContact = Contact - Reach.Start;
        const double Distance = ToContact.Size();
        if (Distance > FMath::Max(1.0f, Settings.HandReachDistance))
        {
            return Reach;
        }

        Reach.Radius = FMath::Max(1.0f, ProbeSettings->GetHandSweepRadius());
        const double Clearance = Reach.Radius + 0.25;
        const FVector Forward = ToContact.GetSafeNormal2D(
            UE_SMALL_NUMBER, Holder.GetActorForwardVector().GetSafeNormal2D());
        const double Rise = FMath::DegreesToRadians(FMath::Clamp<double>(Settings.ProbeRiseDegrees, 0.0, 89.0));
        const FVector FirstDirection = Forward * FMath::Cos(Rise) + FVector::UpVector * FMath::Sin(Rise);
        double FirstLength = FMath::Min<double>(FMath::Max(1.0f, ProbeSettings->GetProbeFirstLength()), 0.75 * Distance);

        const double OriginGap = FVector::DotProduct(Reach.Start - Contact, Normal);
        const double GapChange = FVector::DotProduct(FirstDirection, Normal);
        if (Normal.Z <= -0.6 && OriginGap > Clearance && GapChange < -UE_SMALL_NUMBER)
        {
            // Keep the bend below an underside surface when approaching it from below.
            FirstLength = FMath::Min(FirstLength, (OriginGap - Clearance) / -GapChange);
        }

        Reach.Bend = Reach.Start + FirstDirection * FirstLength;
        // Stop the sphere center outside the face so its surface approaches the intended contact.
        Reach.End = Contact + Normal * Clearance;
        const FCollisionQueryParams Query(SCENE_QUERY_STAT(CarryBentReach), false, &Holder);
        const FCollisionShape Shape = FCollisionShape::MakeSphere(Reach.Radius);
        FHitResult Hit;
        if (World.SweepSingleByChannel(Hit, Reach.Start, Reach.Bend, FQuat::Identity, ECC_Visibility, Shape, Query))
        {
            Reach.End = Hit.Location;
            Reach.BlockedLeg = 1;
            return Reach;
        }

        if (World.SweepSingleByChannel(Hit, Reach.Bend, Reach.End, FQuat::Identity, ECC_Visibility, Shape, Query))
        {
            Reach.End = Hit.Location;
            const bool bReachedGrip = !Hit.bStartPenetrating && Hit.GetComponent() == &Body
                && FVector::DistSquared(Hit.ImpactPoint, Contact) <= FMath::Square(2.0 * Reach.Radius + 1.0);
            Reach.BlockedLeg = bReachedGrip ? 0 : 2;
        }
        return Reach;
    }

#if ENABLE_DRAW_DEBUG
    void DrawReachSweep(const UWorld& World, const FVector& Start, const FVector& End, float Radius, const FColor& Color)
    {
        const FVector Segment = End - Start;
        const double Length = Segment.Size();
        if (Length <= UE_KINDA_SMALL_NUMBER)
        {
            DrawDebugSphere(&World, Start, Radius, 12, Color, false, -1.0f, 0, 1.0f);
            return;
        }

        const FQuat Rotation = FQuat::FindBetweenNormals(FVector::UpVector, Segment / Length);
        DrawDebugCapsule(&World, Start + Segment * 0.5, static_cast<float>(Length * 0.5) + Radius, Radius,
            Rotation, Color, false, -1.0f, 0, 0.0f);
    }

    void DrawBentReachPath(const UWorld& World, const FGripReach& Reach, const FVector& Contact)
    {
        if (Reach.Radius <= 0.0f)
        {
            DrawDebugPoint(&World, Contact, 8.0f, FColor::Red, false, -1.0f);
            return;
        }

        const bool bFirstBlocked = Reach.BlockedLeg == 1;
        DrawReachSweep(World, Reach.Start, bFirstBlocked ? Reach.End : Reach.Bend, Reach.Radius,
            bFirstBlocked ? FColor::Red : FColor(80, 170, 255));
        if (!bFirstBlocked)
        {
            DrawDebugPoint(&World, Reach.Bend, 7.0f, FColor::White, false, -1.0f);
            DrawReachSweep(World, Reach.Bend, Reach.End, Reach.Radius,
                Reach.BlockedLeg == 2 ? FColor::Red : FColor::Magenta);
        }
        DrawDebugPoint(&World, Reach.CanReach() ? Contact : Reach.End, 8.0f,
            Reach.CanReach() ? FColor::Green : FColor::Red, false, -1.0f);
    }
#endif

    void ApplyPredictiveInterpolationCVars()
    {
        static bool bCVarsApplied = false;
        if (bCVarsApplied)
        {
            return;
        }
        bCVarsApplied = true;

        struct FCVarOverride
        {
            const TCHAR* Name;
            const TCHAR* Value;
        };

        // Keep dynamic collisions velocity-driven and prevent aggressive soft-snap / deep target extrapolation into obstacles.
        static constexpr FCVarOverride Overrides[] = {
            { TEXT("np2.PredictiveInterpolation.CorrectionAsVelocity"), TEXT("0") },
            { TEXT("np2.PredictiveInterpolation.DisableSoftSnap"), TEXT("1") },
            { TEXT("np2.PredictiveInterpolation.SoftSnapToSource"), TEXT("1") },
            { TEXT("np2.PredictiveInterpolation.ExtrapolationMinTime"), TEXT("0.05") },
            { TEXT("np2.PredictiveInterpolation.ExtrapolationTimeMultiplier"), TEXT("1.25") },
            { TEXT("np2.PredictiveInterpolation.PosCorrectionTimeBase"), TEXT("0.03") },
            { TEXT("np2.PredictiveInterpolation.PosCorrectionTimeMin"), TEXT("0.10") },
            { TEXT("np2.PredictiveInterpolation.RotCorrectionTimeBase"), TEXT("0.03") },
            { TEXT("np2.PredictiveInterpolation.RotCorrectionTimeMin"), TEXT("0.10") },
            { TEXT("np2.PredictiveInterpolation.InterpolationTimeMultiplier"), TEXT("1.05") },
            { TEXT("np2.PredictiveInterpolation.RotInterpolationTimeMultiplier"), TEXT("1.1") },
        };

        IConsoleManager& ConsoleManager = IConsoleManager::Get();
        for (const FCVarOverride& Override : Overrides)
        {
            if (IConsoleVariable* CVar = ConsoleManager.FindConsoleVariable(Override.Name))
            {
                CVar->Set(Override.Value, ECVF_SetByCode);
            }
        }
    }
}

UCarryableComponent::UCarryableComponent()
{
    SetIsReplicatedByDefault(true);
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

const FCarryItemSettings* UCarryableComponent::GetProfileSettings() const
{
    return ReadCarryProfile<FCarryItemSettings>(ItemProfile.Get());
}

int32 UCarryableComponent::GetEffectiveMaxHolders() const
{
    if (MaxHoldersOverride != 0)
    {
        return FMath::Clamp(MaxHoldersOverride, 1, 4);
    }

    const FCarryItemSettings* Settings = GetProfileSettings();
    return FMath::Clamp(Settings ? Settings->MaxHolders : MaxHolders, 1, 4);
}

void UCarryableComponent::BeginPlay()
{
    Super::BeginPlay();
    ApplyPredictiveInterpolationCVars();
    if (AActor* Owner = GetOwner())
    {
        CachedBody = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
        CachedPhysics = Owner->FindComponentByClass<UCarryPhysicsComponent>();
        CachedCollision = Owner->FindComponentByClass<UCarryCollisionComponent>();
        Owner->SetPhysicsReplicationMode(EPhysicsReplicationMode::PredictiveInterpolation);
    }
    if (const FCarryItemSettings* Settings = GetProfileSettings(); Settings && CachedBody)
    {
        if (Settings->PhysicalMaterial)
        {
            CachedBody->SetPhysMaterialOverride(Settings->PhysicalMaterial.Get());
        }
        if (Settings->bOverrideMass)
        {
            CachedBody->SetMassOverrideInKg(NAME_None, FMath::Max(0.01f, Settings->MassKg));
        }
        if (Settings->bOverrideBodyDamping)
        {
            CachedBody->SetLinearDamping(Settings->BodyLinearDamping);
            CachedBody->SetAngularDamping(Settings->BodyAngularDamping);
        }
    }
    OnRep_Grips();
}

void UCarryableComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(UCarryableComponent, Grips);
}

TArray<int32> UCarryableComponent::GetGripPlayerIds() const
{
    TArray<int32> PlayerIds;
    PlayerIds.Reserve(Grips.Num());
    for (const FCarryGrip& Grip : Grips)
    {
        PlayerIds.Add(Grip.PlayerId);
    }
    return PlayerIds;
}

bool UCarryableComponent::TryAddGrip(APawn& Holder, const FHitResult& Hit)
{
    AActor* Item = GetOwner();
    UPrimitiveComponent* Body = GetBodyComponent();
    if (!Item || !Item->HasAuthority() || !Item->GetIsReplicated() || !Body || Hit.GetComponent() != Body ||
        !Body->IsSimulatingPhysics() || Grips.Num() >= GetEffectiveMaxHolders() || HasGripFor(Holder))
    {
        return false;
    }

    if (!EvaluateGripReach(*Item->GetWorld(), Holder, *Body, Hit.ImpactPoint, Hit.ImpactNormal).CanReach())
    {
        return false;
    }

    Item->FlushNetDormancy();
    Body->WakeAllRigidBodies();
    const FTransform& BodyTransform = Body->GetComponentTransform();
    FCarryGrip& Grip = Grips.AddDefaulted_GetRef();
    Grip.Holder = &Holder;
    if (const APlayerState* PlayerState = Holder.GetPlayerState())
    {
        Grip.PlayerId = PlayerState->GetPlayerId();
    }
    Grip.LocalPoint = BodyTransform.InverseTransformPosition(Hit.ImpactPoint);
    Grip.LocalNormal = BodyTransform.InverseTransformVectorNoScale(Hit.ImpactNormal).GetSafeNormal();
    const FTransform View = GetCarryViewTransform(Holder);
    Grip.ViewLocalTarget = View.InverseTransformPosition(Hit.ImpactPoint);
    Grip.SmoothedTarget = Hit.ImpactPoint;
    const FVector InitialOffset = Hit.ImpactPoint - View.GetLocation();
    Grip.PreviousAimDirection = InitialOffset.GetSafeNormal(UE_SMALL_NUMBER, View.GetRotation().GetForwardVector());
    Grip.SmoothedHoldDistance = InitialOffset.Size();
    Grip.CarryTargetYaw = FMath::Atan2(InitialOffset.Y, InitialOffset.X);
    Grip.SmoothedHorizontalReach = InitialOffset.Size2D();
    Grip.CenterRadiusOffset = static_cast<float>((Body->GetCenterOfMass() - View.GetLocation()).Size2D() - Grip.SmoothedHorizontalReach);
    Grip.InitializedHolder = &Holder;
    Grip.InitializedLocalPoint = Grip.LocalPoint;
    Grip.LastRawViewLocation = View.GetLocation();
    Grip.LastRawViewRotation = View.GetRotation();
    Grip.RemoteViewAngularVelocity = FVector::ZeroVector;
    Grip.TimeSinceViewSample = 0.0;
    Grip.InitialBodyRotation = Body->GetComponentTransform().GetRotation();
    Grip.InitialViewYaw = View.GetRotation().Rotator().Yaw;
    Item->ForceNetUpdate();
    OnRep_Grips();
    return true;
}

void UCarryableComponent::RemoveGrip(const APawn& Holder)
{
    AActor* Item = GetOwner();
    if (!Item || !Item->HasAuthority())
    {
        return;
    }

    const int32 Index = Grips.IndexOfByPredicate([&Holder](const FCarryGrip& Grip) { return Grip.Holder == &Holder; });
    if (Index != INDEX_NONE)
    {
        Item->FlushNetDormancy();
        Grips.RemoveAt(Index);
        Item->ForceNetUpdate();
        OnRep_Grips();
    }
}

bool UCarryableComponent::HasGripFor(const APawn& Holder) const
{
    return Grips.ContainsByPredicate([&Holder](const FCarryGrip& Grip) { return Grip.Holder == &Holder; });
}

bool UCarryableComponent::TryGetGripWorldPoint(const APawn& Holder, FVector& OutPoint) const
{
    const UPrimitiveComponent* Body = GetBodyComponent();
    const FCarryGrip* Grip = Grips.FindByPredicate([&Holder](const FCarryGrip& Entry) { return Entry.Holder == &Holder; });
    if (!IsValid(Body) || !Body->IsSimulatingPhysics() || !Grip)
    {
        return false;
    }

    OutPoint = Body->GetComponentTransform().TransformPosition(Grip->LocalPoint);
    return true;
}

void UCarryableComponent::RemoveInvalidGrips()
{
    AActor* Item = GetOwner();
    if (!Item || !Item->HasAuthority())
    {
        return;
    }

    const int32 Removed = Grips.RemoveAll([](const FCarryGrip& Grip) { return !IsValid(Grip.Holder.Get()); });
    if (Removed > 0)
    {
        Item->FlushNetDormancy();
        Item->ForceNetUpdate();
        OnRep_Grips();
    }
}

UPrimitiveComponent* UCarryableComponent::GetBodyComponent() const
{
    if (CachedBody)
    {
        return CachedBody.Get();
    }
    const AActor* Owner = GetOwner();
    return Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
}

void UCarryableComponent::OnRep_Grips()
{
    const bool bIsHeld = !Grips.IsEmpty();
    if (AActor* Owner = GetOwner())
    {
        Owner->SetNetUpdateFrequency(100.0f);
        if (bIsHeld)
        {
            Owner->SetMinNetUpdateFrequency(60.0f);
        }
    }
    if (UPrimitiveComponent* Body = GetBodyComponent())
    {
        Body->WakeAllRigidBodies();
    }
    SetComponentTickEnabled((bShowGripMarkers || bShowBentReachPreview) && bIsHeld && GetNetMode() != NM_DedicatedServer);
    if (UCarryCollisionComponent* Collision = CachedCollision ? CachedCollision.Get() : (GetOwner() ? GetOwner()->FindComponentByClass<UCarryCollisionComponent>() : nullptr))
    {
        Collision->UpdateHolders(Grips);
    }
    if (UCarryPhysicsComponent* Physics = CachedPhysics ? CachedPhysics.Get() : (GetOwner() ? GetOwner()->FindComponentByClass<UCarryPhysicsComponent>() : nullptr))
    {
        Physics->HandleGripsChanged();
    }
    else if (!bIsHeld)
    {
        if (AActor* Owner = GetOwner())
        {
            Owner->SetMinNetUpdateFrequency(10.0f);
        }
    }
}

void UCarryableComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UWorld* World = GetWorld();
    const UPrimitiveComponent* Body = GetBodyComponent();
    if (!World || !Body)
    {
        return;
    }

    const FTransform& BodyTransform = Body->GetComponentTransform();
    for (const FCarryGrip& Grip : Grips)
    {
        const FVector Contact = BodyTransform.TransformPosition(Grip.LocalPoint);
        const FVector Normal = BodyTransform.TransformVectorNoScale(Grip.LocalNormal).GetSafeNormal();
#if ENABLE_DRAW_DEBUG
        if (bShowBentReachPreview && IsValid(Grip.Holder.Get()))
        {
            const FGripReach Reach = EvaluateGripReach(*World, *Grip.Holder, *Body, Contact, Normal);
            DrawBentReachPath(*World, Reach, Contact);
        }
#endif
        if (!bShowGripMarkers)
        {
            continue;
        }
        constexpr float MarkerRadius = 6.0f;
        const FVector Center = Contact + Normal * MarkerRadius;

        // 1. Grab point marker + surface normal
        DrawDebugSphere(World, Center, MarkerRadius, 12, FColor::Green, false, -1.0f, 0, 1.8f);
        DrawDebugDirectionalArrow(World, Contact, Contact + Normal * 22.0f, 8.0f, FColor::Green, false, -1.0f, 0, 1.8f);

        // 2. Target pull direction vector (Contact -> SmoothedTarget)
        if (!Grip.SmoothedTarget.IsZero())
        {
            const FVector PullVector = Grip.SmoothedTarget - Contact;
            const double PullDist = PullVector.Size();
            if (PullDist > 1.0)
            {
                const FVector PullEnd = Contact + PullVector.GetClampedToMaxSize(100.0);
                DrawDebugDirectionalArrow(World, Contact, PullEnd, 12.0f, FColor::Cyan, false, -1.0f, 0, 1.5f);
            }
            DrawDebugSphere(World, Grip.SmoothedTarget, 3.5f, 8, FColor::Cyan, false, -1.0f, 0, 1.2f);
        }

        // 3. Applied force vector (in Newtons: cm*kg/s^2 -> N)
        const FVector ForceNewtons = Grip.AppliedForce / 100.0;
        const double ForceMagN = ForceNewtons.Size();
        const FVector ForceDir = ForceMagN > UE_KINDA_SMALL_NUMBER ? (ForceNewtons / ForceMagN) : FVector::ZeroVector;
        if (ForceMagN > 0.5)
        {
            const float ForceRatio = FMath::Clamp(static_cast<float>(ForceMagN / 250.0), 0.0f, 1.0f);
            const FColor ForceColor = FLinearColor::LerpUsingHSV(FLinearColor::Yellow, FLinearColor::Red, ForceRatio).ToFColor(true);
            const float ArrowLength = FMath::Clamp(static_cast<float>(ForceMagN * 0.25), 16.0f, 110.0f);
            DrawDebugDirectionalArrow(World, Contact, Contact + ForceDir * ArrowLength, 16.0f, ForceColor, false, -1.0f, 0, 2.5f);
        }

        // 4. Readout label: ID, Force magnitude (N), 3D Force vector, and unit direction
        const FVector LabelPos = Center + FVector::UpVector * (MarkerRadius + 10.0f);
        const FString LabelText = FString::Printf(
            TEXT("ID %d | %.1f N [%.0f, %.0f, %.0f]\nDir (%.2f, %.2f, %.2f)"),
            Grip.PlayerId,
            ForceMagN,
            ForceNewtons.X, ForceNewtons.Y, ForceNewtons.Z,
            ForceDir.X, ForceDir.Y, ForceDir.Z);
        DrawDebugString(World, LabelPos, LabelText, nullptr, FColor::White, 0.0f, true, 1.15f);
    }
}
