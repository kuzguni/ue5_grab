#include "CarryableComponent.h"

#include "Data/CarryReplicationRates.h"
#include "CarryCollisionComponent.h"
#include "CarryPhysicsComponent.h"
#include "Data/CarryProfileBinding.h"
#include "Debug/CarryDebugDraw.h"
#include "CarryProfileSettings.h"
#include "World/CarryWorldQueries.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "HAL/IConsoleManager.h"
#include "Net/UnrealNetwork.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

namespace
{
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
        Owner->SetNetUpdateFrequency(Carry::NetUpdateFrequencyHz);
        if (bIsHeld)
        {
            Owner->SetMinNetUpdateFrequency(Carry::HeldNetUpdateFrequencyHz);
        }
    }
    if (UPrimitiveComponent* Body = GetBodyComponent())
    {
        Body->WakeAllRigidBodies();
    }
    SetComponentTickEnabled((bShowGripMarkers || bShowItemDebug || bShowBentReachPreview) && bIsHeld && GetNetMode() != NM_DedicatedServer);
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
            Owner->SetMinNetUpdateFrequency(Carry::SettledNetUpdateFrequencyHz);
        }
    }
}

#if WITH_EDITOR
void UCarryableComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    if (const UWorld* World = GetWorld(); World && World->IsGameWorld())
    {
        SetComponentTickEnabled((bShowGripMarkers || bShowItemDebug || bShowBentReachPreview)
            && !Grips.IsEmpty() && GetNetMode() != NM_DedicatedServer);
    }
}
#endif

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
    const int32 GripCount = Grips.Num();
    for (int32 GripIndex = 0; GripIndex < GripCount; ++GripIndex)
    {
        const FCarryGrip& Grip = Grips[GripIndex];
        const FVector Contact = BodyTransform.TransformPosition(Grip.LocalPoint);
        const FVector Normal = BodyTransform.TransformVectorNoScale(Grip.LocalNormal).GetSafeNormal();
#if ENABLE_DRAW_DEBUG
        if (bShowBentReachPreview && IsValid(Grip.Holder.Get()))
        {
            const CarryWorld::FGripReach Reach = CarryWorld::EvaluateGripReach(*World, *Grip.Holder, *Body, Contact, Normal);
            CarryDebug::DrawBentReachPath(*World, Reach, Contact);
        }
#endif
        if (bShowGripMarkers)
        {
            CarryDebug::DrawGripMarkers(*World, *Body, Grip, GripIndex, Contact, Normal, GripCount);
        }
    }

    if (bShowItemDebug && GripCount > 0)
    {
        if (const AActor* Owner = GetOwner())
        {
            CarryDebug::DrawItemDebug(*World, *Owner, *Body, Grips, GetEffectiveMaxHolders());
        }
    }
}
