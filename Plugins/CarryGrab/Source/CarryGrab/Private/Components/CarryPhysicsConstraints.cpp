#include "CarryPhysicsComponent.h"

#include "Carry/CarryConstants.h"
#include "Carry/CarryDrive.h"
#include "CarryableComponent.h"
#include "Engine/World.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/PBDJointConstraintData.h"
#include "Chaos/ShapeInstance.h"
#include "Chaos/Sphere.h"
#include "GameFramework/Pawn.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

UCarryPhysicsComponent::FActiveGripConstraint* UCarryPhysicsComponent::FindOrCreateGripConstraint(
    const FCarryGrip& Grip, const APawn& Holder, FPhysScene* Scene, const FPhysicsActorHandle& GrabbedHandle,
    const FTransform& GrabbedWorldTransform, const FVector& BodyContact, const FTransform& KinematicTarget)
{
    FActiveGripConstraint* ActiveConstraint = nullptr;
    for (const TUniquePtr<FActiveGripConstraint>& Existing : ActiveConstraints)
    {
        if (Existing && Existing->Holder == &Holder && Existing->LocalPoint.Equals(Grip.LocalPoint, Carry::GripAnchorToleranceCm))
        {
            ActiveConstraint = Existing.Get();
            break;
        }
    }

    if (!ActiveConstraint)
    {
        TUniquePtr<FActiveGripConstraint> NewEntry = MakeUnique<FActiveGripConstraint>();
        NewEntry->Holder = &Holder;
        NewEntry->LocalPoint = Grip.LocalPoint;

        const FTransform InitialTargetPose(KinematicTarget.GetRotation(), BodyContact);
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
    return ActiveConstraint;
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
                return Grip.Holder == Entry->Holder && Grip.LocalPoint.Equals(Entry->LocalPoint, Carry::GripAnchorToleranceCm);
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

bool UCarryPhysicsComponent::UpdateGripConstraintDrive(FActiveGripConstraint& ActiveConstraint, FCarryGrip& Grip,
    UPrimitiveComponent& Body, const UCarryableComponent& Item, const FBodyDrive& Drive, const FHolderDrive& HolderDrive,
    const FTransform& GrabbedWorldTransform, const FVector& BodyLocation, const FVector& BodyContact,
    const FTransform& KinematicTarget, double YawAlignVelZ)
{
    const double VerticalForceCap = HolderDrive.MaxVerticalForce;
    const FQuat ConnectorWorldRot = GrabbedWorldTransform.GetRotation() * ActiveConstraint.GrabbedLocalFrame.GetRotation();
    const FMatrix ConnectorAxesMatrix = ConnectorWorldRot.ToMatrix();
    FVector AxisMaxForce(HolderDrive.PerHorizCap, HolderDrive.PerHorizCap, HolderDrive.MaxForce);
    double SumVertCap = 0.0;
    for (int32 AxisIdx = 0; AxisIdx < 3; ++AxisIdx)
    {
        const FVector AxisW = ConnectorAxesMatrix.GetScaledAxis(static_cast<EAxis::Type>(EAxis::X + AxisIdx));
        const double HorizSq = AxisW.X * AxisW.X + AxisW.Y * AxisW.Y;
        const double VertSq = AxisW.Z * AxisW.Z;
        const double InvCapSq = HorizSq / FMath::Square(FMath::Max(HolderDrive.PerHorizCap, 1.0))
            + VertSq / FMath::Square(FMath::Max(VerticalForceCap, 1.0));
        const double Cap = InvCapSq > UE_SMALL_NUMBER ? 1.0 / FMath::Sqrt(InvCapSq) : HolderDrive.MaxForce;
        AxisMaxForce[AxisIdx] = Cap;
        SumVertCap += FMath::Abs(AxisW.Z) * Cap;
    }
    if (SumVertCap > VerticalForceCap && SumVertCap > UE_SMALL_NUMBER)
    {
        // Preserve axes perpendicular to world up when limiting the combined vertical projection.
        for (int32 AxisIdx = 0; AxisIdx < 3; ++AxisIdx)
        {
            const FVector AxisW = ConnectorAxesMatrix.GetScaledAxis(static_cast<EAxis::Type>(EAxis::X + AxisIdx));
            if (FMath::Abs(AxisW.Z) > UE_SMALL_NUMBER)
            {
                AxisMaxForce[AxisIdx] *= VerticalForceCap / SumVertCap;
            }
        }
    }

    if (Grip.RegripTime >= 0.0)
    {
        constexpr double RegripBlendTime = 0.12;
        AxisMaxForce *= FMath::SmoothStep(0.0, RegripBlendTime,
            GetWorld()->GetTimeSeconds() - Grip.RegripTime + Drive.StepDt);
    }

    // Ramp the temporary pose drive in and out. The regular angular velocity drive still controls yaw.
    constexpr double HoldTiltEngageRate = 12.0;
    constexpr double HoldTiltReleaseRate = 5.0;
    Grip.SwingHoldAlpha = FMath::FInterpConstantTo(Grip.SwingHoldAlpha, Grip.bSuppressSwing ? 1.0 : 0.0,
        Drive.StepDt, Grip.bSuppressSwing ? HoldTiltEngageRate : HoldTiltReleaseRate);
    const double HoldTorque = Grip.SwingHoldAlpha * HolderDrive.AngularTorqueCapChaos * HolderDrive.AngularOrientScale;
    const bool bHoldTilt = HoldTorque > UE_SMALL_NUMBER;
    FTransform JointTarget = KinematicTarget;
    JointTarget.SetLocation(LimitJointTarget(KinematicTarget.GetLocation(), Body, Item, BodyLocation, BodyContact,
        Drive.MassPerHolder, Drive.LinearStiffnessChaos));
    if (bHoldTilt)
    {
        const FQuat RotationFromHold = GrabbedWorldTransform.GetRotation() * Grip.SwingHoldRotation.Inverse();
        FQuat Tilt, Yaw;
        RotationFromHold.ToSwingTwist(FVector::UpVector, Tilt, Yaw);
        JointTarget.SetRotation(Yaw * Grip.SwingHoldRotation * ActiveConstraint.GrabbedLocalFrame.GetRotation());
    }
    constexpr double HoldTiltErrorDegrees = 5.0;
    const double PoseStiffness = HoldTorque / FMath::DegreesToRadians(HoldTiltErrorDegrees);
    const double PoseDamping = 2.0 * Drive.DriveDampingRatio * FMath::Sqrt(PoseStiffness * Drive.AvgInertia);
    const double AngularDamping = FMath::Max(Drive.AngularDampingChaos, PoseDamping);
    const FVector LocalAngVelTarget = ConnectorWorldRot.UnrotateVector(FVector(0.0, 0.0, -YawAlignVelZ));

    FPhysicsCommand::ExecuteWrite(ActiveConstraint.ConstraintHandle, [&](const FPhysicsConstraintHandle& InConstraint)
    {
        FPhysicsInterface::SetKinematicTarget_AssumesLocked(ActiveConstraint.KinematicHandle, JointTarget);
        if (Chaos::FJointConstraint* Constraint = static_cast<Chaos::FJointConstraint*>(InConstraint.Constraint))
        {
            Constraint->SetLinearPositionDriveEnabled(Chaos::TVec3<bool>(HolderDrive.MaxForce > 0.0));
            Constraint->SetLinearVelocityDriveEnabled(Chaos::TVec3<bool>(HolderDrive.MaxForce > 0.0));
            Constraint->SetLinearDriveStiffness(Chaos::FVec3(Drive.LinearStiffnessChaos));
            Constraint->SetLinearDriveDamping(Chaos::FVec3(Drive.LinearDampingChaos));
            Constraint->SetLinearDriveMaxForce(Chaos::FVec3(AxisMaxForce.X, AxisMaxForce.Y, AxisMaxForce.Z));

            Constraint->SetAngularSLerpPositionDriveEnabled(bHoldTilt);
            Constraint->SetAngularSLerpVelocityDriveEnabled(HolderDrive.AngularTorqueCapChaos > 0.0);
            Constraint->SetAngularDriveVelocityTarget(Chaos::FVec3(LocalAngVelTarget.X, LocalAngVelTarget.Y, LocalAngVelTarget.Z));
            Constraint->SetAngularDriveStiffness(Chaos::FVec3(PoseStiffness));
            Constraint->SetAngularDriveDamping(Chaos::FVec3(AngularDamping));
            Constraint->SetAngularDriveMaxTorque(Chaos::FVec3(HolderDrive.AngularTorqueCapChaos));
        }
    });

    FVector RawLinImpulse = FVector::ZeroVector;
    FVector RawAngImpulse = FVector::ZeroVector;
    FPhysicsInterface::GetForce(ActiveConstraint.ConstraintHandle, RawLinImpulse, RawAngImpulse);
    const FVector SolverForce = -RawLinImpulse / Drive.StepDt;
    const FVector SolverTorque = -RawAngImpulse / Drive.StepDt;
    if (SolverForce.SizeSquared() > 1.0 || SolverTorque.SizeSquared() > 1.0)
    {
        Grip.AppliedForce = SolverForce.GetClampedToMaxSize(HolderDrive.MaxForce);
        Grip.AppliedTorque = SolverTorque.GetClampedToMaxSize(HolderDrive.AngularTorqueCapChaos);
        return true;
    }
    return false;
}
