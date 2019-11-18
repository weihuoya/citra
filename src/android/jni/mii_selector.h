#pragma once

#include <vector>

#include "core/frontend/applets/mii_selector.h"

class AndroidMiiSelector final : public Frontend::MiiSelector {
public:
    AndroidMiiSelector();
    void Setup(const Frontend::MiiSelectorConfig& config) override;

    void LoadExtSaveData();

private:
    std::vector<HLE::Applets::MiiData> miis;
};
