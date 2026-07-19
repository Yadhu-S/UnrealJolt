#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

class FUnrealJoltEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
