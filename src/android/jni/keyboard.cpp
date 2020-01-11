#include "jni_common.h"
#include "keyboard.h"

AndroidKeyboard::AndroidKeyboard() = default;

void AndroidKeyboard::Execute(const Frontend::KeyboardConfig& config) {
    SoftwareKeyboard::Execute(config);
    std::string button0, button1, button2;
    if (config.button_config == Frontend::ButtonConfig::None) {
        //
    } else if (config.button_config == Frontend::ButtonConfig::Single) {
        if (config.has_custom_button_text) {
            button0 = config.button_text[0];
        }
        if (button0.empty()) {
            button0 = Frontend::SWKBD_BUTTON_OKAY;
        }
    } else if (config.button_config == Frontend::ButtonConfig::Dual) {
        if (config.has_custom_button_text) {
            button0 = config.button_text[1];
            button1 = config.button_text[0];
        }
        if (button0.empty()) {
            button0 = Frontend::SWKBD_BUTTON_OKAY;
        }
        if (button1.empty()) {
            button1 = Frontend::SWKBD_BUTTON_CANCEL;
        }
    } else if (config.button_config == Frontend::ButtonConfig::Triple) {
        if (config.has_custom_button_text) {
            button0 = config.button_text[2];
            button1 = config.button_text[1];
            button2 = config.button_text[0];
        } else {
            button0 = Frontend::SWKBD_BUTTON_OKAY;
            button1 = Frontend::SWKBD_BUTTON_FORGOT;
            button2 = Frontend::SWKBD_BUTTON_CANCEL;
        }
        if (button0.empty()) {
            button0 = Frontend::SWKBD_BUTTON_OKAY;
        }
        if (button1.empty()) {
            button1 = Frontend::SWKBD_BUTTON_FORGOT;
        }
        if (button2.empty()) {
            button2 = Frontend::SWKBD_BUTTON_CANCEL;
        }
    }
    if (config.button_config != Frontend::ButtonConfig::None) {
        mButtonOk = static_cast<u8>(config.button_config);
    }
    ShowInputBoxDialog(config.max_text_length, config.hint_text, button0, button1, button2);
}

void AndroidKeyboard::ShowError(const std::string& error) {
    ShowMessageDialog(0, error);
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
