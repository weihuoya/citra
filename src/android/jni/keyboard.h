#pragma once

#include <string>
#include "core/frontend/applets/swkbd.h"

class AndroidKeyboard final : public Frontend::SoftwareKeyboard {
public:
    AndroidKeyboard();
    void Execute(const Frontend::KeyboardConfig& config) override;
    void ShowError(const std::string& error) override;

    // from java
    void Accept(int action, const std::string& text);

private:
    std::string mError;
    u8 mButtonOk;
};
