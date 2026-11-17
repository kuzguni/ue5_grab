#pragma once

#include "Carry/CarryTarget.h"
#include "CarryPhysicsComponent.h"

struct UCarryPhysicsComponent::FBodyDrive
{
    Carry::FTargetFollow Follow;
    double StepDt;
    double MassPerHolder;
    double GravityZ;
    double AvgInertia;
    double DriveDampingRatio;
    double LinearStiffnessChaos;
    double LinearDampingChaos;
    double AngularDampingChaos;
    double AccelerationCap;
    double TurnFraction;
    double LoadedTurnScale;
    double TurnLoadAlpha;
    double GyrationRadiusCm;
};

struct UCarryPhysicsComponent::FHolderDrive
{
    FVector PerHolderGravityHold;
    double MaxForce;
    double MaxVerticalForce;
    double FullForceExtension;
    double TurningAcceleration;
    double AngularOrientScale;
    double AngularTorqueCapChaos;
    double PerHorizCap;
};
