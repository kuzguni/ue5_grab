#pragma once

#include "Camera/CameraComponent.h"
#include "GameFramework/Pawn.h"

inline FTransform GetCarryViewTransform(const APawn& Pawn)
{
    if (Pawn.bFindCameraComponentWhenViewTarget)
    {
        TInlineComponentArray<UCameraComponent*> Cameras;
        Pawn.GetComponents(Cameras);
        for (const UCameraComponent* Camera : Cameras)
        {
            if (!Camera->IsActive())
            {
                continue;
            }

            // Remote cameras are not rotated by the local camera manager.
            // Use control rotation (or replicated aim for simulated pawns) in that case too.
            const FRotator ViewRotation = Camera->bUsePawnControlRotation
                ? (Pawn.GetController() ? Pawn.GetViewRotation() : Pawn.GetBaseAimRotation())
                : Camera->GetComponentRotation();
            return FTransform(ViewRotation, Camera->GetComponentLocation());
        }
    }

    return FTransform(Pawn.GetBaseAimRotation(), Pawn.GetPawnViewLocation());
}
