#pragma once

#include "CoreMinimal.h"

namespace JoltCallbackOrder
{
	inline constexpr int32 Early = -100;
	inline constexpr int32 Default = 0;
	inline constexpr int32 Late = 100;
}

struct FJoltOrderedCallbacks
{
	struct FEntry
	{
		int32				   Order = JoltCallbackOrder::Default;
		TDelegate<void(float)> Callback;
	};

	uint16 Add(const TDelegate<void(float)>& Callback, int32 InOrder)
	{
		int32 Index = Entries.Num();
		// Walk back past every entry with a larger order
		while (Index > 0 && Entries[Index - 1].Order > InOrder)
		{
			--Index;
		}
		Entries.Insert({ InOrder, Callback }, Index);
		return static_cast<uint16>(Index);
	}

	void Execute(float DeltaSeconds) const
	{
		for (const FEntry& Entry : Entries)
		{
			if (Entry.Callback.IsBound())
			{
				Entry.Callback.Execute(DeltaSeconds);
			}
		}
	}

private:
	TArray<FEntry> Entries;
};
