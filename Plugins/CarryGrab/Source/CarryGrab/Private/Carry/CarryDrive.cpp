#include "Carry/CarryDrive.h"

#include "Carry/CarryConstants.h"
#include "CarryableComponent.h"
#include "CarryProfileSettings.h"

namespace
{
    constexpr double DampingRatio = 0.9;
    constexpr double LookSpeedDecayRate = 12.0;
    constexpr double SwingStartSpeed = 60.0; // cm/s of target motion from turning the view
    constexpr double DeliberateSwingSpeed = 260.0;
    constexpr double TurnAccelerationFraction = 0.27;
    constexpr double TurnBrakingTime = 0.38; // seconds from full sideways speed to rest
    constexpr double MovingAngularDampingScale = 0.55;
    constexpr double ReferenceAngularDampingScale = 1.25;
    constexpr double HeavyAngularDampingScale = 1.5;
    constexpr double CompactDampingFullRadiusCm = 6.0;
    constexpr double CompactDampingEndRadiusCm = 12.0;
}

UCarryPhysicsComponent::FBodyDrive UCarryPhysicsComponent::CalculateBodyDrive(const TArray<FCarryGrip>& Grips,
    int32 HolderCount, const FCarryItemSettings* Settings, double Mass, double GravityZ, double AvgInertia,
    double StepDt) const
{
    const double MassPerHolder = Mass / HolderCount;
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
    const Carry::FTargetFollow Follow{ StepDt, BrakingTime, LookSpeedDecay, FollowScale, TargetAlpha };

    const double GyrationRadiusCm = FMath::Sqrt(AvgInertia / Mass);

    double FastestLookSpeed = 0.0;
    for (const FCarryGrip& Grip : Grips)
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
    // Small collision shapes gain more angular speed from a similar off-center acceleration.
    const double CompactAlpha = 1.0 - FMath::SmoothStep(CompactDampingFullRadiusCm, CompactDampingEndRadiusCm, GyrationRadiusCm);
    const double CompactDamping = FMath::Clamp<double>(Settings ? Settings->CompactSwingDampingScale : 3.0f, 1.0, 5.0);
    const double SettleRate = Settings ? Settings->AngularSettleRate : AngularSettleRate;
    const double AngularDampingChaos = DriveDampingRatio * SettleRate * AngularDampingScale * AvgInertia
        * FMath::Lerp(1.0, CompactDamping, CompactAlpha);
    return { Follow, StepDt, MassPerHolder, GravityZ, AvgInertia, DriveDampingRatio, LinearStiffnessChaos,
        LinearDampingChaos, AngularDampingChaos, AccelerationCap, TurnFraction, LoadedTurnScale, TurnLoadAlpha,
        GyrationRadiusCm };
}

UCarryPhysicsComponent::FHolderDrive UCarryPhysicsComponent::CalculateHolderDrive(const FBodyDrive& Drive,
    bool bSolo, double ForceNewtons, double TorqueNm)
{
    const double MaxForce = Carry::CentimetersPerMeter * FMath::Max(0.0, FMath::Min(ForceNewtons, Drive.MassPerHolder * Drive.AccelerationCap));
    const double GravitySupportForce = FMath::Clamp(-Drive.MassPerHolder * Drive.GravityZ, 0.0, MaxForce);
    const FVector PerHolderGravityHold(0.0, 0.0, GravitySupportForce);
    const double FullForceExtension = MaxForce / Drive.LinearStiffnessChaos;
    const double TurningAcceleration = Drive.TurnFraction * MaxForce / Drive.MassPerHolder * FMath::Lerp(1.0, Drive.LoadedTurnScale, Drive.TurnLoadAlpha);
    const double WeightRatio = (-Drive.MassPerHolder * Drive.GravityZ) / FMath::Max(MaxForce, 1.0);
    const double AngularOrientScale = bSolo ? 1.0 - FMath::SmoothStep(0.85, 1.15, WeightRatio) : 0.0;
    const double AngularTorqueCapChaos = FMath::Min(FMath::Max(0.0, TorqueNm) * FMath::Square(Carry::CentimetersPerMeter), 0.5 * MaxForce * Drive.GyrationRadiusCm);
    const double RemainingHorizSq = FMath::Max(0.0, FMath::Square(MaxForce) - FMath::Square(GravitySupportForce));
    const double PerHorizCap = FMath::Max(0.45 * MaxForce, FMath::Sqrt(RemainingHorizSq)) / UE_SQRT_2;
    return { PerHolderGravityHold, MaxForce, FullForceExtension, TurningAcceleration, AngularOrientScale,
        AngularTorqueCapChaos, PerHorizCap };
}
