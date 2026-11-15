#pragma once

#include "Engine/DataAsset.h"
#include "UObject/UnrealType.h"

// Blueprint profile types expose an instance-editable Settings variable.
template <typename T>
const T* ReadCarryProfile(const UPrimaryDataAsset* Asset)
{
    if (Asset)
    {
        const FStructProperty* Field = FindFProperty<FStructProperty>(Asset->GetClass(), TEXT("Settings"));
        if (Field && Field->Struct == T::StaticStruct())
        {
            return Field->ContainerPtrToValuePtr<T>(Asset);
        }
    }
    return nullptr;
}
