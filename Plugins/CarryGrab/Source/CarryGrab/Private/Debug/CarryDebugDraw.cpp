#include "Debug/CarryDebugDraw.h"

#include "Carry/CarryConstants.h"
#include "CarryableComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "SceneTypes.h"
#include "World/CarryWorldQueries.h"

namespace CarryDebug
{
    namespace
    {
        constexpr double TorqueScaleFactor = Carry::CentimetersPerMeter * Carry::CentimetersPerMeter;

        double CleanZero(double Value, double Tolerance)
        {
            return FMath::Abs(Value) < Tolerance ? 0.0 : Value;
        }

        FString FormatVec0(const FVector& Vec)
        {
            return FString::Printf(TEXT("[%+.0f,%+.0f,%+.0f]"),
                CleanZero(Vec.X, 0.5), CleanZero(Vec.Y, 0.5), CleanZero(Vec.Z, 0.5));
        }

        FString FormatVec1(const FVector& Vec)
        {
            return FString::Printf(TEXT("[%+.1f,%+.1f,%+.1f]"),
                CleanZero(Vec.X, 0.05), CleanZero(Vec.Y, 0.05), CleanZero(Vec.Z, 0.05));
        }

#if ENABLE_DRAW_DEBUG
        constexpr double MinVisibleYawDegS = 3.0;
        const FColor RotationDebugColor(245, 105, 255);
        const FColor GripHeaderColor(210, 245, 255);
        const FColor ItemHeaderColor(255, 225, 130);
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

        void DrawYawArc(const UWorld& World, const FVector& Center, double YawDegS, float Radius)
        {
            constexpr int32 Segments = 20;
            constexpr float Thickness = 2.4f;
            const FVector U0 = FVector::ForwardVector;
            const FVector U1 = FVector::RightVector * FMath::Sign(YawDegS);
            const double SweepRad = FMath::DegreesToRadians(240.0);
            // The center of mass is often inside the mesh.
            const auto DrawArcLine = [&World](const FVector& Start, const FVector& End)
            {
                DrawDebugLine(&World, Start, End, RotationDebugColor, false, -1.0f, SDPG_Foreground, Thickness);
            };
            FVector PrevPoint = Center + U0 * Radius;
            for (int32 Step = 1; Step <= Segments; ++Step)
            {
                const double Angle = SweepRad * (static_cast<double>(Step) / Segments);
                const FVector NextPoint = Center + (FMath::Cos(Angle) * U0 + FMath::Sin(Angle) * U1) * Radius;
                DrawArcLine(PrevPoint, NextPoint);
                PrevPoint = NextPoint;
            }

            const FVector RadialEnd = FMath::Cos(SweepRad) * U0 + FMath::Sin(SweepRad) * U1;
            const FVector TangentEnd = (-FMath::Sin(SweepRad) * U0 + FMath::Cos(SweepRad) * U1).GetSafeNormal();
            const float HeadSize = FMath::Clamp(Radius * 0.38f, 4.0f, 8.5f);
            DrawArcLine(PrevPoint, PrevPoint + (-TangentEnd + RadialEnd * 0.55) * HeadSize);
            DrawArcLine(PrevPoint, PrevPoint + (-TangentEnd - RadialEnd * 0.55) * HeadSize);
        }

        FString BuildRotationStatusText(const FCarryGrip& Grip, int32 GripCount)
        {
            const double CmdDegS = CleanZero(FMath::RadiansToDegrees(Grip.YawAlignVelZ), 0.5);
            const double AngleLeftDeg = CleanZero(FMath::RadiansToDegrees(Grip.TargetYawErrorRad), 0.05);
            FString Text;
            if (Grip.bHasManualYawTarget && Grip.RotationInput != 0)
            {
                Text = FString::Printf(TEXT("Rot [Manual %+d]: Cmd %+.0f d/s | Rem %+.1f d"),
                    Grip.RotationInput, CmdDegS, AngleLeftDeg);
            }
            else if (Grip.bHasManualYawTarget)
            {
                Text = FString::Printf(TEXT("Rot [HoldYaw]: Cmd %+.0f d/s | Rem %+.1f d"),
                    CmdDegS, AngleLeftDeg);
            }
            else if (Grip.bUsingViewYawFollow)
            {
                Text = FString::Printf(TEXT("Rot [Follow]: Cmd %+.0f d/s | Rem %+.1f d"),
                    CmdDegS, AngleLeftDeg);
            }
            else if (GripCount > 1)
            {
                Text = TEXT("Rot [Co-op Damp]: Cmd +0 d/s");
            }
            else if (Grip.AngularOrientScale <= UE_KINDA_SMALL_NUMBER)
            {
                Text = TEXT("Rot [Heavy Damp]: Cmd +0 d/s");
            }
            else
            {
                Text = TEXT("Rot [Idle Damp]: Cmd +0 d/s");
            }

            if (Grip.SwingHoldAlpha > 0.01)
            {
                Text += FString::Printf(TEXT(" | TiltHold %.0f%%"), Grip.SwingHoldAlpha * 100.0);
            }
            return Text;
        }
#endif
    }

#if ENABLE_DRAW_DEBUG
    void DrawBentReachPath(const UWorld& World, const CarryWorld::FGripReach& Reach, const FVector& Contact)
    {
        if (Reach.Radius <= 0.0f)
        {
            DrawDebugPoint(&World, Contact, 8.0f, FColor::Red, false, -1.0f);
            return;
        }

        const bool bFirstBlocked = Reach.Block == CarryWorld::EReachBlock::FirstSweep;
        DrawReachSweep(World, Reach.Start, bFirstBlocked ? Reach.End : Reach.Bend, Reach.Radius,
            bFirstBlocked ? FColor::Red : FColor(80, 170, 255));
        if (!bFirstBlocked)
        {
            DrawDebugPoint(&World, Reach.Bend, 7.0f, FColor::White, false, -1.0f);
            DrawReachSweep(World, Reach.Bend, Reach.End, Reach.Radius,
                Reach.Block == CarryWorld::EReachBlock::SecondSweep ? FColor::Red : FColor::Magenta);
        }
        DrawDebugPoint(&World, Reach.CanReach() ? Contact : Reach.End, 8.0f,
            Reach.CanReach() ? FColor::Green : FColor::Red, false, -1.0f);

        const TCHAR* ReachState = Reach.CanReach()
            ? TEXT("Reach: OK")
            : (bFirstBlocked ? TEXT("Reach: BLOCKED (Sweep 1)") : TEXT("Reach: BLOCKED (Sweep 2)"));
        DrawDebugString(&World, Contact + FVector::UpVector * 30.0, ReachState,
            nullptr, Reach.CanReach() ? FColor::Green : FColor::Red, 0.0f, true, 1.05f);
    }
#endif

    void DrawGripMarkers(const UWorld& World, const UPrimitiveComponent& Body, const FCarryGrip& Grip,
        int32 GripIndex, const FVector& Contact, const FVector& Normal, int32 GripCount)
    {
#if ENABLE_DRAW_DEBUG
        const FVector CoM = Body.GetCenterOfMass();
        const FTransform& BodyTransform = Body.GetComponentTransform();

        // 1. Grab point marker + surface normal (Green) + CoM lever arm link (Gray)
        DrawDebugSphere(&World, Contact, 3.5f, 10, FColor::Green, false, -1.0f, 0, 1.5f);
        DrawDebugDirectionalArrow(&World, Contact, Contact + Normal * 14.0f, 6.0f, FColor::Green, false, -1.0f, 0, 1.5f);
        if (FVector::DistSquared(Contact, CoM) > 4.0)
        {
            DrawDebugLine(&World, CoM, Contact, FColor(160, 160, 160), false, -1.0f, 0, 0.75f);
        }

        // 2. Target pull error (Cyan)
        double PullDist = 0.0;
        if (!Grip.SmoothedTarget.IsZero())
        {
            const FVector PullVector = Grip.SmoothedTarget - Contact;
            PullDist = PullVector.Size();
            if (PullDist > 1.0)
            {
                const FVector PullEnd = Contact + PullVector.GetClampedToMaxSize(100.0);
                DrawDebugDirectionalArrow(&World, Contact, PullEnd, 10.0f, FColor::Cyan, false, -1.0f, 0, 1.4f);
            }
            DrawDebugSphere(&World, Grip.SmoothedTarget, 3.0f, 8, FColor::Cyan, false, -1.0f, 0, 1.2f);
        }

        // 3. Applied linear force vector (Yellow-Green -> Red by utilization of this grip's cap)
        const FVector ForceNewtons = Grip.AppliedForce / Carry::CentimetersPerMeter;
        const double ForceMagN = ForceNewtons.Size();
        const double MaxForceN = Grip.MaxForce / Carry::CentimetersPerMeter;
        const double ForceLeftN = FMath::Max(0.0, MaxForceN - ForceMagN);
        if (ForceMagN > 0.5)
        {
            const FVector ForceDir = ForceNewtons / ForceMagN;
            const float ForceRatio = MaxForceN > UE_SMALL_NUMBER
                ? FMath::Clamp(static_cast<float>(ForceMagN / MaxForceN), 0.0f, 1.0f)
                : FMath::Clamp(static_cast<float>(ForceMagN / 250.0), 0.0f, 1.0f);
            const FColor ForceColor = FLinearColor::LerpUsingHSV(
                FLinearColor(0.35f, 1.0f, 0.2f), FLinearColor::Red, ForceRatio).ToFColor(true);
            const float ArrowLength = 18.0f + 72.0f * ForceRatio;
            DrawDebugDirectionalArrow(&World, Contact, Contact + ForceDir * ArrowLength, 14.0f, ForceColor, false, -1.0f, 0, 2.4f);
        }

        // 4. Grip drive torque readout
        const FVector DriveTorqueNm = Grip.AppliedTorque / TorqueScaleFactor;
        const double DriveTorqueMagNm = DriveTorqueNm.Size();
        const double MaxTorqueNm = Grip.MaxTorque / TorqueScaleFactor;
        const double TorqueLeftNm = FMath::Max(0.0, MaxTorqueNm - DriveTorqueMagNm);

        // 5. Off-center lever torque from linear force around CoM
        const FVector LeverArmMeters = (Contact - CoM) / Carry::CentimetersPerMeter;
        const FVector LeverTorqueNm = FVector::CrossProduct(LeverArmMeters, ForceNewtons);
        const double LeverTorqueMagNm = LeverTorqueNm.Size();
        const FVector GripTotalTorqueNm = DriveTorqueNm + LeverTorqueNm;

        // Place grip readout outward from CoM so multiple grips and the item header stay separated.
        FVector Outward(Contact.X - CoM.X, Contact.Y - CoM.Y, (Contact.Z - CoM.Z) * 0.35);
        if (Outward.SizeSquared2D() < 4.0)
        {
            const FVector Side = BodyTransform.GetRotation().GetRightVector() * ((GripIndex % 2 == 0) ? 1.0 : -1.0);
            Outward = Side + FVector(Normal.X, Normal.Y, 0.0) * 0.5;
        }
        Outward.Normalize();
        const FVector LabelPos = Contact + Outward * 24.0 + FVector::UpVector * 8.0;

        const FString ForceText = FString::Printf(
            TEXT("G%d (P%d) | Err: %.1f cm\n")
            TEXT("Force: %.0f/%.0f N (Left %.0f) %s"),
            GripIndex + 1,
            Grip.PlayerId,
            PullDist,
            ForceMagN, MaxForceN, ForceLeftN, *FormatVec0(ForceNewtons));
        DrawDebugString(&World, LabelPos, ForceText, nullptr, GripHeaderColor, 0.0f, true, 1.0f);

        const FString RotLine = BuildRotationStatusText(Grip, GripCount);
        const FString RotationText = FString::Printf(
            TEXT("\n\n%s\n")
            TEXT("DrvTrq: %.1f/%.1f Nm (Left %.1f) %s\n")
            TEXT("LevTrq: %.1f Nm %s | Tot: %.1f Nm"),
            *RotLine,
            DriveTorqueMagNm, MaxTorqueNm, TorqueLeftNm, *FormatVec1(DriveTorqueNm),
            LeverTorqueMagNm, *FormatVec1(LeverTorqueNm), GripTotalTorqueNm.Size());
        DrawDebugString(&World, LabelPos, RotationText, nullptr, RotationDebugColor, 0.0f, true, 1.0f);
#endif
    }

    void DrawItemDebug(const UWorld& World, const AActor& Owner, const UPrimitiveComponent& Body,
        const TArray<FCarryGrip>& Grips, int32 EffectiveMaxHolders)
    {
#if ENABLE_DRAW_DEBUG
        const FVector CoM = Body.GetCenterOfMass();
        const FTransform& BodyTransform = Body.GetComponentTransform();
        const FRotator BodyRot = BodyTransform.GetRotation().Rotator();
        const double YawDegS = Body.GetPhysicsAngularVelocityInDegrees().Z;

        const double MassKg = FMath::Max(static_cast<double>(Body.GetMass()), UE_SMALL_NUMBER);
        const double GravityZ = World.GetGravityZ();
        const FVector WeightN(0.0, 0.0, MassKg * GravityZ / Carry::CentimetersPerMeter);
        const double WeightMagN = FMath::Abs(WeightN.Z);

        FVector SumGripForceN = FVector::ZeroVector;
        double SumForceUsedN = 0.0;
        double SumForceCapN = 0.0;

        FVector SumDriveTorqueNm = FVector::ZeroVector;
        FVector SumLeverTorqueNm = FVector::ZeroVector;
        double SumDriveTorqueUsedNm = 0.0;
        double SumDriveTorqueCapNm = 0.0;

        TArray<FString, TInlineAllocator<4>> GripTorqueBreakdown;
        int32 ValidGrips = 0;

        for (int32 Index = 0; Index < Grips.Num(); ++Index)
        {
            const FCarryGrip& Grip = Grips[Index];
            if (!IsValid(Grip.Holder.Get()))
            {
                continue;
            }
            ++ValidGrips;

            const FVector Contact = BodyTransform.TransformPosition(Grip.LocalPoint);
            const FVector ForceN = Grip.AppliedForce / Carry::CentimetersPerMeter;
            const double MaxForceN = Grip.MaxForce / Carry::CentimetersPerMeter;
            SumGripForceN += ForceN;
            SumForceUsedN += ForceN.Size();
            SumForceCapN += MaxForceN;

            const FVector DriveTorqueNm = Grip.AppliedTorque / TorqueScaleFactor;
            const double MaxTorqueNm = Grip.MaxTorque / TorqueScaleFactor;
            const FVector LeverArmM = (Contact - CoM) / Carry::CentimetersPerMeter;
            const FVector LeverTorqueNm = FVector::CrossProduct(LeverArmM, ForceN);

            SumDriveTorqueNm += DriveTorqueNm;
            SumLeverTorqueNm += LeverTorqueNm;
            SumDriveTorqueUsedNm += DriveTorqueNm.Size();
            SumDriveTorqueCapNm += MaxTorqueNm;

            const FVector GripTotalTorqueNm = DriveTorqueNm + LeverTorqueNm;
            GripTorqueBreakdown.Add(FString::Printf(TEXT("G%d %s"), Index + 1, *FormatVec1(GripTotalTorqueNm)));
        }

        if (ValidGrips == 0)
        {
            return;
        }

        const FVector NetForceN = SumGripForceN + WeightN;
        const double NetForceMagN = NetForceN.Size();
        const double ForceCapLeftN = FMath::Max(0.0, SumForceCapN - SumForceUsedN);

        const FVector NetTorqueNm = SumDriveTorqueNm + SumLeverTorqueNm;
        const double NetTorqueMagNm = NetTorqueNm.Size();
        const double DriveTorqueCapLeftNm = FMath::Max(0.0, SumDriveTorqueCapNm - SumDriveTorqueUsedNm);

        // CoM marker + net linear force arrow (white) + measured yaw arc (magenta)
        DrawDebugSphere(&World, CoM, 4.0f, 10, FColor(255, 210, 60), false, -1.0f, 0, 1.8f);
        if (NetForceMagN > 2.0)
        {
            const float NetArrowLen = FMath::Clamp(static_cast<float>(NetForceMagN * 0.18), 16.0f, 95.0f);
            DrawDebugDirectionalArrow(&World, CoM, CoM + (NetForceN / NetForceMagN) * NetArrowLen,
                14.0f, FColor::White, false, -1.0f, 0, 2.2f);
        }
        if (FMath::Abs(YawDegS) >= MinVisibleYawDegS)
        {
            const float ArcRadius = FMath::Clamp(static_cast<float>(Body.Bounds.SphereRadius * 0.45f), 18.0f, 28.0f);
            DrawYawArc(World, CoM, YawDegS, ArcRadius);
        }

        const FBoxSphereBounds Bounds = Body.Bounds;
        const FVector ItemLabelPos = FVector(CoM.X, CoM.Y, Bounds.Origin.Z + Bounds.BoxExtent.Z) + FVector::UpVector * 50.0;
        const TCHAR* SourceTag = Owner.GetLocalRole() != ROLE_Authority ? TEXT(" [Client Est]") : TEXT("");

        const FString ItemForceText = FString::Printf(
            TEXT("[%s%s] %.1f kg (W %.0f N) | Grips %d/%d\n")
            TEXT("Forces: Effort %.0f/%.0f N (Left %.0f N)\n")
            TEXT("  GripsSum: %.0f N %s | Net(+W): %.0f N %s"),
            *Owner.GetActorNameOrLabel(), SourceTag, MassKg, WeightMagN, ValidGrips, FMath::Clamp(EffectiveMaxHolders, 1, 4),
            SumForceUsedN, SumForceCapN, ForceCapLeftN,
            SumGripForceN.Size(), *FormatVec0(SumGripForceN), NetForceMagN, *FormatVec0(NetForceN));
        DrawDebugString(&World, ItemLabelPos, ItemForceText, nullptr, ItemHeaderColor, 0.0f, true, 1.05f);

        FString ItemRotationText = FString::Printf(
            TEXT("\n\n\nPose: P%.1f Y%.1f R%.1f | Yaw rate: %+.0f d/s\n")
            TEXT("Torque XYZ (Nm): DrvCap Left %.1f/%.1f\n")
            TEXT("  Drv %s + Lev %s\n")
            TEXT("  => Net Rem: %.1f Nm %s"),
            BodyRot.Pitch, BodyRot.Yaw, BodyRot.Roll, CleanZero(YawDegS, 0.5),
            DriveTorqueCapLeftNm, SumDriveTorqueCapNm,
            *FormatVec1(SumDriveTorqueNm), *FormatVec1(SumLeverTorqueNm),
            NetTorqueMagNm, *FormatVec1(NetTorqueNm));

        if (ValidGrips >= 2)
        {
            ItemRotationText += FString::Printf(TEXT("\n  Grips: %s => Rem %s"),
                *FString::Join(GripTorqueBreakdown, TEXT(" + ")),
                *FormatVec1(NetTorqueNm));
        }

        DrawDebugString(&World, ItemLabelPos, ItemRotationText, nullptr, RotationDebugColor, 0.0f, true, 1.05f);
#endif
    }
}
