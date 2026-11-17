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
        constexpr double MinTurnRadius = 8.0; // cm, below this yaw has little physical effect
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
            Grip.LastRawViewLocation = ViewTransform.GetLocation();
            Grip.LastRawViewRotation = ViewTransform.GetRotation();
            Grip.RemoteViewAngularVelocity = FVector::ZeroVector;
            Grip.TimeSinceViewSample = 0.0;
            Grip.bStabilizeTurnDistance = false;
            Grip.bEaseOutFastTurn = false;
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
        FVector TargetOffset = ViewOffset;
        const double TurnRadius = ViewOffset.Size2D();
        const double DesiredYaw = TurnRadius >= 1.0
            ? FMath::Atan2(ViewOffset.Y, ViewOffset.X)
            : FMath::DegreesToRadians(ViewTransform.GetRotation().Rotator().Yaw);
        if (TurnRadius >= MinTurnRadius)
        {
            // v^2 / r is the inward acceleration needed to keep following this arc.
            // Limit how fast the target sweeps sideways so the grip has force left to pull inward.
            const double MaxYawSpeed = FMath::Sqrt(TurningAcceleration / TurnRadius);
            const double YawError = FMath::FindDeltaAngleRadians(Grip.CarryTargetYaw, DesiredYaw);
            const double YawDistance = FMath::Abs(YawError);
            const bool bReachedTurnLimit = YawDistance > MaxYawSpeed * Follow.StepDt;
            Grip.bStabilizeTurnDistance |= bReachedTurnLimit;
            Grip.bEaseOutFastTurn |= bReachedTurnLimit;

            // Start braking while there is still an arc left to travel. Using v^2 = 2*a*d
            // ramps down the target's sideways speed instead of stopping it in one tick.
            const double ArrivalSpeed = Grip.bEaseOutFastTurn
                ? FMath::Sqrt(2.0 * (MaxYawSpeed / Follow.BrakingTime) * YawDistance)
                : MaxYawSpeed;
            const double StepLimit = FMath::Min(MaxYawSpeed, ArrivalSpeed) * Follow.StepDt;
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
            const double ObstacleBackoff = 1.0 - FMath::SmoothStep(UE_PI / 8.0, UE_PI / 5.0, FMath::Abs(YawError));
            YawAlignVelZ = AngularOrientScale * (4.0 * BoundedError + EffectiveYawRateRad * ObstacleBackoff);
        }
        else if (Target.MotionAlpha <= UE_KINDA_SMALL_NUMBER || AngularOrientScale <= UE_KINDA_SMALL_NUMBER)
        {
            Grip.InitialBodyRotation = BodyRotation;
            Grip.InitialViewYaw = Target.YawDegrees;
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
