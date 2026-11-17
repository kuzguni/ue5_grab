#include "Debug/CarryDebugDraw.h"

#include "Carry/CarryConstants.h"
#include "CarryableComponent.h"
#include "World/CarryWorldQueries.h"

namespace CarryDebug
{
#if ENABLE_DRAW_DEBUG
    namespace
    {
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
    }

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
    }
#endif

    void DrawGripMarkers(const UWorld& World, const FCarryGrip& Grip, const FVector& Contact, const FVector& Normal)
    {
        constexpr float MarkerRadius = 6.0f;
        const FVector Center = Contact + Normal * MarkerRadius;

        // 1. Grab point marker + surface normal
        DrawDebugSphere(&World, Center, MarkerRadius, 12, FColor::Green, false, -1.0f, 0, 1.8f);
        DrawDebugDirectionalArrow(&World, Contact, Contact + Normal * 22.0f, 8.0f, FColor::Green, false, -1.0f, 0, 1.8f);

        // 2. Target pull direction vector (Contact -> SmoothedTarget)
        if (!Grip.SmoothedTarget.IsZero())
        {
            const FVector PullVector = Grip.SmoothedTarget - Contact;
            const double PullDist = PullVector.Size();
            if (PullDist > 1.0)
            {
                const FVector PullEnd = Contact + PullVector.GetClampedToMaxSize(100.0);
                DrawDebugDirectionalArrow(&World, Contact, PullEnd, 12.0f, FColor::Cyan, false, -1.0f, 0, 1.5f);
            }
            DrawDebugSphere(&World, Grip.SmoothedTarget, 3.5f, 8, FColor::Cyan, false, -1.0f, 0, 1.2f);
        }

        // 3. Applied force vector (in Newtons: cm*kg/s^2 -> N)
        const FVector ForceNewtons = Grip.AppliedForce / Carry::CentimetersPerMeter;
        const double ForceMagN = ForceNewtons.Size();
        const FVector ForceDir = ForceMagN > UE_KINDA_SMALL_NUMBER ? (ForceNewtons / ForceMagN) : FVector::ZeroVector;
        if (ForceMagN > 0.5)
        {
            const float ForceRatio = FMath::Clamp(static_cast<float>(ForceMagN / 250.0), 0.0f, 1.0f);
            const FColor ForceColor = FLinearColor::LerpUsingHSV(FLinearColor::Yellow, FLinearColor::Red, ForceRatio).ToFColor(true);
            const float ArrowLength = FMath::Clamp(static_cast<float>(ForceMagN * 0.25), 16.0f, 110.0f);
            DrawDebugDirectionalArrow(&World, Contact, Contact + ForceDir * ArrowLength, 16.0f, ForceColor, false, -1.0f, 0, 2.5f);
        }

        // 4. Readout label: ID, Force magnitude (N), 3D Force vector, and unit direction
        const FVector LabelPos = Center + FVector::UpVector * (MarkerRadius + 10.0f);
        const FString LabelText = FString::Printf(
            TEXT("ID %d | %.1f N [%.0f, %.0f, %.0f]\nDir (%.2f, %.2f, %.2f)"),
            Grip.PlayerId,
            ForceMagN,
            ForceNewtons.X, ForceNewtons.Y, ForceNewtons.Z,
            ForceDir.X, ForceDir.Y, ForceDir.Z);
        DrawDebugString(&World, LabelPos, LabelText, nullptr, FColor::White, 0.0f, true, 1.15f);
    }
}
