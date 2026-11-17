#pragma once

#include "Math/Quat.h"
#include "Math/Transform.h"
#include "Math/Vector.h"

class UPlayerCarryComponent;
class APawn;
struct FCarryGrip;
struct FCarryPlayerSettings;

namespace Carry
{
    struct FTargetFollow
    {
        double StepDt;
        double BrakingTime;
        double LookSpeedDecay;
        double FollowScale;
        double TargetAlpha;
    };

    struct FGripTarget
    {
        FVector Velocity;
        double YawDegrees;
        double MotionAlpha;
    };

    FGripTarget UpdateGripTarget(FCarryGrip& Grip, const APawn& Holder, const FTransform& ViewTransform,
        const FQuat& BodyRotation, float HoldDistanceFollowRate, const FTargetFollow& Follow,
        double TurningAcceleration);

    double UpdateGripYaw(FCarryGrip& Grip, const FGripTarget& Target, const FTargetFollow& Follow, bool bSolo,
        const FQuat& BodyRotation, double AngularOrientScale, const UPlayerCarryComponent* PlayerCarry,
        const FCarryPlayerSettings* PlayerSettings);
}
