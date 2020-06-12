#include "jni_common.h"
#include "keyboard.h"

AndroidKeyboard::AndroidKeyboard() = default;

void AndroidKeyboard::Execute(const Frontend::KeyboardConfig& config) {
    SoftwareKeyboard::Execute(config);
    std::string button0, button1, button2;
    switch (config.button_config) {
    case Frontend::ButtonConfig::Triple:
        button1 = config.button_text[1].empty() ? Frontend::SWKBD_BUTTON_FORGOT : config.button_text[1];
    case Frontend::ButtonConfig::Dual:
        button1 = config.button_text[0].empty() ? Frontend::SWKBD_BUTTON_CANCEL : config.button_text[0];
    case Frontend::ButtonConfig::Single:
        button1 = config.button_text[2].empty() ? Frontend::SWKBD_BUTTON_OKAY : config.button_text[2];
    case Frontend::ButtonConfig::None:
        break;
    }
    if (config.button_config != Frontend::ButtonConfig::None) {
        mButtonOk = static_cast<u8>(config.button_config);
    }
    ShowInputBoxDialog(config.max_text_length, mError, config.hint_text, button0, button1, button2);
    mError.clear();
}

void AndroidKeyboard::ShowError(const std::string& error) {
    mError = error;
}

void AndroidKeyboard::Accept(int action, const std::string& text) {
    const int EVENT_FINISH = -1;
    const int EVENT_BUTTON0 = 0;
    const int EVENT_BUTTON1 = 1;
    const int EVENT_BUTTON2 = 2;

    constexpr u8 forgot_id = 1;
    constexpr u8 cancel_id = 0;

    Frontend::ValidationError error = Frontend::ValidationError::None;
    if (EVENT_FINISH == action || EVENT_BUTTON0 == action) {
        error = Finalize(text, mButtonOk);
    } else if (EVENT_BUTTON1 == action) {
        error = Finalize(text, forgot_id);
    } else if (EVENT_BUTTON2 == action) {
        error = Finalize(text, cancel_id);
    }

    if (error != Frontend::ValidationError::None) {
        Execute(config);
    }
}
