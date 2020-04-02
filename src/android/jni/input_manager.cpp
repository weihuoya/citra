#include "input_manager.h"

#include <tuple>

#include <android/log.h>
#include <fmt/format.h>

#include "core/frontend/input.h"
#include "core/settings.h"
#include "input_common/keyboard.h"
#include "input_common/main.h"
#include "input_common/motion_emu.h"

#include "config/main_settings.h"

static const char* TOUCHSCREEN = "touchscreen";
static const int REVERSE_KEY_START = 4096;
#define MAX_ANALOG_VALUE 1.64F

class Analog final : public Input::AnalogDevice {
public:
    explicit Analog(int axisX, int axisY, float maxValue)
        : mAxisX(axisX), mAxisY(axisY), mMaxValue(maxValue) {}

    std::tuple<float, float> GetStatus() const override {
        float axisX = InputManager::GetInstance().GetInput(mAxisX);
        float axisY = InputManager::GetInstance().GetInput(mAxisY);

        if (axisX > mMaxValue)
            axisX = mMaxValue;
        else if (axisX < -mMaxValue)
            axisX = -mMaxValue;

        if (axisY > mMaxValue)
            axisY = mMaxValue;
        else if (axisY < -mMaxValue)
            axisY = -mMaxValue;

        return std::make_tuple(axisX, axisY);
    }

private:
    int mAxisX;
    int mAxisY;
    float mMaxValue;
};

class AnalogFactory final : public Input::Factory<Input::AnalogDevice> {
public:
    std::unique_ptr<Input::AnalogDevice> Create(const Common::ParamPackage& params) override {
        int axisX = params.Get("axis_x", 0);
        int axisY = params.Get("axis_y", 0);
        float maxValue = params.Get("max_value", MAX_ANALOG_VALUE);
        return std::make_unique<Analog>(axisX, axisY, maxValue);
    }
};

InputManager& InputManager::GetInstance() {
    static InputManager s_input_manager;
    return s_input_manager;
}

InputManager::InputManager() {
    InputCommon::Init();
    Input::RegisterFactory<Input::AnalogDevice>(TOUCHSCREEN, std::make_shared<AnalogFactory>());
}

InputManager::~InputManager() {
    Input::UnregisterFactory<Input::AnalogDevice>(TOUCHSCREEN);
    InputCommon::Shutdown();
}

void InputManager::InitProfile() {
    Settings::values.current_input_profile_index = 0;
    Settings::InputProfile profile;
    profile.name = TOUCHSCREEN;

    std::array<std::string, Settings::NativeButton::NumButtons> buttonConfigs = {
        Config::Get(Config::BUTTON_A),     Config::Get(Config::BUTTON_B),
        Config::Get(Config::BUTTON_X),     Config::Get(Config::BUTTON_Y),
        Config::Get(Config::BUTTON_UP),    Config::Get(Config::BUTTON_DOWN),
        Config::Get(Config::BUTTON_LEFT),  Config::Get(Config::BUTTON_RIGHT),
        Config::Get(Config::BUTTON_L),     Config::Get(Config::BUTTON_R),
        Config::Get(Config::BUTTON_START), Config::Get(Config::BUTTON_SELECT),
        Config::Get(Config::BUTTON_DEBUG), Config::Get(Config::BUTTON_GPIO14),
        Config::Get(Config::BUTTON_ZL),    Config::Get(Config::BUTTON_ZR),
        Config::Get(Config::BUTTON_HOME),
    };

    mButtonKeys.clear();
    for (int i = 0; i < buttonConfigs.size(); ++i) {
        const std::string& config = buttonConfigs[i];
        Common::ParamPackage param(config);
        int keyCode = param.Get("code", i);
        if (param.Get("dir", "+") == "-") {
            keyCode = REVERSE_KEY_START + keyCode;
            param.Set("code", keyCode);
        }
        param.Set("engine", "keyboard");
        profile.buttons[i] = param.Serialize();
        mButtonKeys.push_back(keyCode);
    }

    std::array<std::string, 8> CirclePadConfigs = {
        Config::Get(Config::CIRCLE_PAD_UP),   Config::Get(Config::CIRCLE_PAD_DOWN),
        Config::Get(Config::CIRCLE_PAD_LEFT), Config::Get(Config::CIRCLE_PAD_RIGHT),
        Config::Get(Config::C_STICK_UP), Config::Get(Config::C_STICK_DOWN),
        Config::Get(Config::C_STICK_LEFT), Config::Get(Config::C_STICK_RIGHT),
    };

    mCirclePadKeys.clear();
    for (int i = 0; i < CirclePadConfigs.size(); ++i) {
        const std::string& config = CirclePadConfigs[i];
        Common::ParamPackage param(config);
        int keyCode = param.Get("code", i);
        if (param.Get("dir", "+") == "-") {
            keyCode = REVERSE_KEY_START + keyCode;
        }
        mCirclePadKeys.push_back(keyCode);
    }

    mAnalogs.resize(4);
    profile.analogs[Settings::NativeAnalog::CirclePad] =
        fmt::format("axis_x:0,axis_y:1,engine:{}", TOUCHSCREEN);
    profile.analogs[Settings::NativeAnalog::CStick] =
        fmt::format("axis_x:2,axis_y:3,max_value:1,engine:{}", TOUCHSCREEN);

    profile.motion_device = "engine:motion_emu";
    profile.touch_device = "engine:emu_window";
    profile.udp_input_address = "127.0.0.1";
    profile.udp_input_port = 26760;
    profile.udp_pad_index = 0;

    Settings::values.input_profiles.clear();
    Settings::values.input_profiles.emplace_back(std::move(profile));
    Settings::LoadProfile(Settings::values.current_input_profile_index);
}

float InputManager::GetInput(int button) {
    return mAnalogs[button];
}

void InputManager::InputEvent(int button, float value) {
    if (button >= N3DS_CPAD_X) {
        mAnalogs[button - N3DS_CPAD_X] = value;
    } else {
        KeyEvent(mButtonKeys[button], value);
    }
}

bool InputManager::KeyEvent(int button, float value) {
    if (value < 0) {
        value = -value;
        button = REVERSE_KEY_START + button;
    }

    // buttons
    for (auto code : mButtonKeys) {
        if (code == button) {
            if (value >= 1.0f) {
                InputCommon::GetKeyboard()->PressKey(button);
            } else {
                InputCommon::GetKeyboard()->ReleaseKey(button);
                InputCommon::GetKeyboard()->ReleaseKey(REVERSE_KEY_START + button);
            }
            return true;
        }
    }

    // circle pad
    std::array<int, 8> circle_pads = {2, -2, -1, 1, 4, -4, -3, 3};
    for (u32 i = 0; i < mCirclePadKeys.size(); ++i) {
        if (mCirclePadKeys[i] == button) {
            auto flag = circle_pads[i];
            if (flag > 0) {
                mAnalogs[flag - 1] = value * MAX_ANALOG_VALUE;
            } else {
                mAnalogs[(-flag) - 1] = -value * MAX_ANALOG_VALUE;
            }
            return true;
        }
    }

    return false;
}

void InputManager::BeginTilt(int x, int y) {
    InputCommon::GetMotionEmu()->BeginTilt(x, y);
}

void InputManager::Tilt(int x, int y) {
    InputCommon::GetMotionEmu()->Tilt(x, y);
}

void InputManager::EndTilt() {
    InputCommon::GetMotionEmu()->EndTilt();
}
