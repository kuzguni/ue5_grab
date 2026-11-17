#include "Carry/CarryTarget.h"

#include "Carry/CarryConstants.h"
#include "CarryableComponent.h"
#include "CarryProfileSettings.h"
#include "GameFramework/Pawn.h"
#include "PlayerCarryComponent.h"

namespace Carry
{
    namespace
    {
        constexpr double MinTurnRadius = 8.0; // cm, below this turning has little physical effect

        double CalculateTurnStep(double Error, double MaxSpeed, const FTargetFollow& Follow,
            bool bHoldFastTurnState, bool& bEaseOut, double ExtraSpeedFraction)
        {
            const double Distance = FMath::Abs(Error);
            bEaseOut |= Distance > MaxSpeed * Follow.StepDt;

            // Brake along the remaining arc using v^2 = 2*a*d.
            const double ArrivalSpeed = bEaseOut
                ? FMath::Sqrt(2.0 * (MaxSpeed / Follow.BrakingTime) * Distance)
                : MaxSpeed;
            const double StepLimit = FMath::Min(MaxSpeed, ArrivalSpeed) * Follow.StepDt;
            // A bounded extra step keeps yaw responsive on fast turns.
            const double ExcessStep = FMath::Max(0.0, Distance - StepLimit);
            const double ExtraSpeedBudget = ExtraSpeedFraction * StepLimit;
            const double ExtraStep = ExcessStep * ExtraSpeedBudget
                / FMath::Max(ExcessStep + ExtraSpeedBudget, UE_SMALL_NUMBER);
            const double Step = FMath::Sign(Error) * (FMath::Min(Distance, StepLimit) + ExtraStep);
            if (!bHoldFastTurnState)
            {
                bEaseOut &= !FMath::IsNearlyEqual(Step, Error);
            }
            return Step;
        }
    }

    FGripTarget UpdateGripTarget(FCarryGrip& Grip, const APawn& Holder, const FTransform& ViewTransform,
        const FQuat& BodyRotation, float HoldDistanceFollowRate, const FTargetFollow& Follow,
        double TurningAcceleration)
    {
        const FVector AimDirection = ViewTransform.TransformVectorNoScale(
            Grip.ViewLocalTarget.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector));
        const double DesiredHoldDistance = Grip.ViewLocalTarget.Size();
        if (Grip.InitializedHolder != &Holder || !Grip.InitializedLocalPoint.Equals(Grip.LocalPoint, Carry::GripAnchorToleranceCm))
        {
            Grip.InitializedHolder = &Holder;
            Grip.InitializedLocalPoint = Grip.LocalPoint;
            const FVector InitialOffset = AimDirection * DesiredHoldDistance;
            Grip.SmoothedTarget = ViewTransform.GetLocation() + InitialOffset;
            Grip.PreviousAimDirection = AimDirection;
            Grip.SmoothedHoldDistance = DesiredHoldDistance;
            Grip.CarryTargetYaw = InitialOffset.Size2D() >= 1.0
                ? FMath::Atan2(InitialOffset.Y, InitialOffset.X)
                : FMath::DegreesToRadians(ViewTransform.GetRotation().Rotator().Yaw);
            Grip.SmoothedHorizontalReach = InitialOffset.Size2D();
            Grip.CarryTargetPitch = FMath::Atan2(InitialOffset.Z, Grip.SmoothedHorizontalReach);
            Grip.LastRawViewLocation = ViewTransform.GetLocation();
            Grip.LastRawViewRotation = ViewTransform.GetRotation();
            Grip.RemoteViewAngularVelocity = FVector::ZeroVector;
            Grip.TimeSinceViewSample = 0.0;
            Grip.bStabilizeTurnDistance = false;
            Grip.bEaseOutFastTurn = false;
            Grip.bEaseOutFastPitch = false;
            Grip.RecentLookSpeed = 0.0;
            Grip.InitialBodyRotation = BodyRotation;
            Grip.InitialViewYaw = FMath::RadiansToDegrees(Grip.CarryTargetYaw);
            Grip.PreviousRotationInput = 0;
            Grip.RotationHoldDuration = 0.0;
        }

        const double LocDeltaSq = FVector::DistSquared(ViewTransform.GetLocation(), Grip.LastRawViewLocation);
        const double RotDelta = Grip.LastRawViewRotation.AngularDistance(ViewTransform.GetRotation());
        const bool bRawViewChanged = Holder.IsLocallyControlled() || LocDeltaSq > 1e-4 || RotDelta > 1e-5;

        const double DistanceRate = FMath::Max(1.0, static_cast<double>(HoldDistanceFollowRate));
        const double DistanceAlpha = 1.0 - FMath::Exp(-DistanceRate * Follow.FollowScale * Follow.StepDt);
        Grip.SmoothedHoldDistance = FMath::Lerp(Grip.SmoothedHoldDistance, DesiredHoldDistance, DistanceAlpha);
        const FVector ViewOffset = AimDirection * Grip.SmoothedHoldDistance;
        bool bHoldFastTurnState = false;
        if (bRawViewChanged)
        {
            const double SampleDt = Holder.IsLocallyControlled()
                ? Follow.StepDt
                : FMath::Clamp(Grip.TimeSinceViewSample + Follow.StepDt, Follow.StepDt, 0.1);
            Grip.LastRawViewLocation = ViewTransform.GetLocation();
            Grip.LastRawViewRotation = ViewTransform.GetRotation();
            Grip.TimeSinceViewSample = 0.0;
            const double LookSpeed = (AimDirection - Grip.PreviousAimDirection).Size() * Grip.SmoothedHoldDistance / SampleDt;
            Grip.PreviousAimDirection = AimDirection;
            Grip.RecentLookSpeed = FMath::Max(LookSpeed, Grip.RecentLookSpeed * Follow.LookSpeedDecay);
        }
        else
        {
            Grip.TimeSinceViewSample += Follow.StepDt;
            Grip.RecentLookSpeed *= Follow.LookSpeedDecay;
            bHoldFastTurnState = Grip.TimeSinceViewSample <= 0.025;
        }

        const FVector PreviousTarget = Grip.SmoothedTarget;
        const double DesiredHorizontalReach = ViewOffset.Size2D();
        const double DesiredPitch = FMath::Atan2(ViewOffset.Z, DesiredHorizontalReach);
        const double HoldDistance = Grip.SmoothedHoldDistance;
        if (HoldDistance >= MinTurnRadius && TurningAcceleration > 0.0)
        {
            // Pitch sweeps through the full hold radius. Limit the arc before setting height and reach.
            const double MaxPitchSpeed = FMath::Sqrt(TurningAcceleration / HoldDistance);
            Grip.CarryTargetPitch += CalculateTurnStep(DesiredPitch - Grip.CarryTargetPitch, MaxPitchSpeed,
                Follow, bHoldFastTurnState, Grip.bEaseOutFastPitch, 0.0);
        }
        else
        {
            Grip.CarryTargetPitch = DesiredPitch;
            Grip.bEaseOutFastPitch = false;
        }

        const double TurnRadius = HoldDistance * FMath::Cos(Grip.CarryTargetPitch);
        const double DesiredYaw = DesiredHorizontalReach >= 1.0
            ? FMath::Atan2(ViewOffset.Y, ViewOffset.X)
            : FMath::DegreesToRadians(ViewTransform.GetRotation().Rotator().Yaw);
        if (TurnRadius >= MinTurnRadius)
        {
            // v^2 / r is the inward acceleration needed to keep following this arc.
            // Limit how fast the target sweeps sideways so the grip has force left to pull inward.
            const double MaxYawSpeed = FMath::Sqrt(TurningAcceleration / TurnRadius);
            const double YawError = FMath::FindDeltaAngleRadians(Grip.CarryTargetYaw, DesiredYaw);
            Grip.bStabilizeTurnDistance |= FMath::Abs(YawError) > MaxYawSpeed * Follow.StepDt;
            const double YawStep = CalculateTurnStep(YawError, MaxYawSpeed, Follow, bHoldFastTurnState,
                Grip.bEaseOutFastTurn, 0.5);
            Grip.CarryTargetYaw = FMath::UnwindRadians(Grip.CarryTargetYaw + YawStep);
        }
        else
        {
            Grip.CarryTargetYaw = DesiredYaw;
            Grip.bEaseOutFastTurn = false;
        }

        const FVector TargetOffset(TurnRadius * FMath::Cos(Grip.CarryTargetYaw),
            TurnRadius * FMath::Sin(Grip.CarryTargetYaw), HoldDistance * FMath::Sin(Grip.CarryTargetPitch));
        const FVector Target = ViewTransform.GetLocation() + TargetOffset;
        Grip.SmoothedTarget = FMath::Lerp(PreviousTarget, Target, Follow.TargetAlpha);
        Grip.SmoothedHorizontalReach = FMath::Lerp(Grip.SmoothedHorizontalReach, TurnRadius, Follow.TargetAlpha);

        const FVector TargetVelocity = (Grip.SmoothedTarget - PreviousTarget) / Follow.StepDt;
        const double HorizontalMotionSpeed = FMath::Max(TargetVelocity.Size2D(), Grip.RecentLookSpeed);
        const double MotionAlpha = FMath::SmoothStep(10.0, 80.0, HorizontalMotionSpeed);
        const double CurrentTargetYawDeg = FMath::RadiansToDegrees(Grip.CarryTargetYaw);
        return { TargetVelocity, CurrentTargetYawDeg, MotionAlpha };
    }

    double UpdateGripYaw(FCarryGrip& Grip, const FGripTarget& Target, const FTargetFollow& Follow, bool bSolo,
        const FQuat& BodyRotation, double AngularOrientScale, const UPlayerCarryComponent* PlayerCarry,
        const FCarryPlayerSettings* PlayerSettings)
    {
        if (!bSolo)
        {
            Grip.RotationInput = 0;
            Grip.PreviousRotationInput = 0;
            Grip.RotationHoldDuration = 0.0;
            Grip.bHasManualYawTarget = false;
            Grip.bSuppressSwing = false;
            Grip.bRegrippedWhileBlocked = false;
            Grip.ClearReachSince = -1.0;
            Grip.SwingHoldAlpha = 0.0;
            Grip.TargetYawErrorRad = 0.0;
            Grip.bUsingViewYawFollow = false;
        }
        if (Grip.RotationInput != 0 && !Grip.bHasManualYawTarget)
        {
            Grip.bHasManualYawTarget = true;
            Grip.ManualYawTarget = BodyRotation;
        }

        double YawAlignVelZ = 0.0;
        if (Grip.bHasManualYawTarget)
        {
            if (Grip.RotationInput == 0)
            {
                Grip.PreviousRotationInput = 0;
                Grip.RotationHoldDuration = 0.0;
            }
            else if (Grip.RotationInput != Grip.PreviousRotationInput)
            {
                Grip.PreviousRotationInput = Grip.RotationInput;
                Grip.RotationHoldDuration = 0.0;
            }
            else
            {
                Grip.RotationHoldDuration += Follow.StepDt;
            }

            const bool bAccelerate = PlayerCarry ? PlayerCarry->GetAccelerateRotation() : true;
            const double DefaultSpeed = PlayerSettings ? static_cast<double>(PlayerSettings->RotationSpeedDegrees) : 70.0;
            const double StartSpeedDeg = FMath::Max(0.0, PlayerCarry ? static_cast<double>(PlayerCarry->GetRotationStartSpeedDegrees()) : DefaultSpeed);
            const double MaxSpeedDeg = FMath::Max(StartSpeedDeg, PlayerCarry ? static_cast<double>(PlayerCarry->GetRotationMaxSpeedDegrees()) : 220.0);
            const double RampSeconds = FMath::Max(0.05, PlayerCarry ? static_cast<double>(PlayerCarry->GetRotationRampTimeSeconds()) : 1.5);
            double RotationSpeedDeg = StartSpeedDeg;
            if (bAccelerate && Grip.RotationInput != 0)
            {
                const double RampAlpha = FMath::Clamp(Grip.RotationHoldDuration / RampSeconds, 0.0, 1.0);
                const double SmoothAlpha = RampAlpha * RampAlpha * (3.0 - 2.0 * RampAlpha);
                RotationSpeedDeg = FMath::Lerp(StartSpeedDeg, MaxSpeedDeg, SmoothAlpha);
            }

            const double EffectiveYawRateRad = FMath::DegreesToRadians(RotationSpeedDeg) * Follow.FollowScale * Grip.RotationInput;
            const double YawStep = EffectiveYawRateRad * Follow.StepDt;
            Grip.ManualYawTarget = FQuat(FVector::UpVector, YawStep) * Grip.ManualYawTarget;
            const FQuat Error = Grip.ManualYawTarget * BodyRotation.Inverse();
            FQuat Swing, Twist;
            Error.ToSwingTwist(FVector::UpVector, Swing, Twist);
            if (Twist.W < 0.0)
            {
                Twist = -Twist;
            }
            const double YawError = FMath::UnwindRadians(2.0 * FMath::Atan2(Twist.Z, Twist.W));
            const double BoundedError = FMath::Clamp(YawError, -UE_PI / 5.0, UE_PI / 5.0);
            // Avoid storing up a large rotation behind an obstacle while the button is held.
            Grip.ManualYawTarget = FQuat(FVector::UpVector, BoundedError - YawError) * Grip.ManualYawTarget;
            Grip.ManualYawTarget.Normalize();
            Grip.TargetYawErrorRad = BoundedError;
            Grip.bUsingViewYawFollow = false;
            const double ObstacleBackoff = 1.0 - FMath::SmoothStep(UE_PI / 8.0, UE_PI / 5.0, FMath::Abs(YawError));
            YawAlignVelZ = AngularOrientScale * (4.0 * BoundedError + EffectiveYawRateRad * ObstacleBackoff);
        }
        else if (Target.MotionAlpha <= UE_KINDA_SMALL_NUMBER || AngularOrientScale <= UE_KINDA_SMALL_NUMBER)
        {
            Grip.InitialBodyRotation = BodyRotation;
            Grip.InitialViewYaw = Target.YawDegrees;
            Grip.TargetYawErrorRad = 0.0;
            Grip.bUsingViewYawFollow = false;
        }
        else
        {
            const double DeltaViewYawDeg = FMath::FindDeltaAngleDegrees(Grip.InitialViewYaw, Target.YawDegrees);
            const FQuat YawCarriedRefQuat = FRotator(0.0, DeltaViewYawDeg, 0.0).Quaternion() * Grip.InitialBodyRotation;
            const FQuat ToTargetQuat = YawCarriedRefQuat * BodyRotation.Inverse();
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

            Grip.TargetYawErrorRad = ClampedYawErrorRad;
            Grip.bUsingViewYawFollow = true;
            const double SwingAngleRad = WorldSwingQuat.GetAngle();
            const double SwingValidity = FMath::SmoothStep(2.2, 1.4, SwingAngleRad);
            constexpr double YawCatchupGain = 3.0;
            constexpr double MaxYawCatchupSpeed = 2.5;
            YawAlignVelZ = AngularOrientScale * Target.MotionAlpha * SwingValidity
                * FMath::Clamp(ClampedYawErrorRad * YawCatchupGain, -MaxYawCatchupSpeed, MaxYawCatchupSpeed);
        }
        return YawAlignVelZ;
    }
}
