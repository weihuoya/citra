#include "input_manager.h"

#include <tuple>

#include <android/log.h>
#include <fmt/format.h>

#include "core/frontend/input.h"
#include "core/settings.h"

static const char* TOUCHSCREEN = "touchscreen";

class Button final : public Input::ButtonDevice {
public:
    explicit Button(int button) : mButton(button) {}

    bool GetStatus() const override {
        return InputManager::GetInstance().GetInput(mButton) == 1.0F;
    }

private:
    int mButton;
};

class Analog final : public Input::AnalogDevice {
public:
    explicit Analog(int axisX, int axisY) : mAxisX(axisX), mAxisY(axisY) {}

    std::tuple<float, float> GetStatus() const override {
        float axisX = InputManager::GetInstance().GetInput(mAxisX);
        float axisY = InputManager::GetInstance().GetInput(mAxisY);
        return std::make_tuple(axisX, axisY);
    }

private:
    int mAxisX;
    int mAxisY;
};

class Motion final : public Input::MotionDevice {
public:
    explicit Motion() {}

    std::tuple<Common::Vec3<float>, Common::Vec3<float>> GetStatus() const override {
        return std::make_tuple<Common::Vec3<float>, Common::Vec3<float>>(
            Common::Vec3<float>(0.0F, 0.0F, 0.0F), Common::Vec3<float>(0.0F, 0.0F, 0.0F));
    }
};

class Touch final : public Input::TouchDevice {
public:
    explicit Touch(int axisX, int axisY, int axisZ) : mAxisX(axisX), mAxisY(axisY), mAxisZ(axisZ) {}

    std::tuple<float, float, bool> GetStatus() const override {
        float axisX = InputManager::GetInstance().GetInput(mAxisX);
        float axisY = InputManager::GetInstance().GetInput(mAxisY);
        float axisZ = InputManager::GetInstance().GetInput(mAxisZ);
        return std::make_tuple(axisX, axisY, axisZ == 1.0F);
    }

private:
    int mAxisX;
    int mAxisY;
    int mAxisZ;
};

class ButtonFactory final : public Input::Factory<Input::ButtonDevice> {
public:
    std::unique_ptr<Input::ButtonDevice> Create(const Common::ParamPackage& params) override {
        int button = params.Get("button", 0);
        return std::make_unique<Button>(button);
    }
};

class AnalogFactory final : public Input::Factory<Input::AnalogDevice> {
public:
    std::unique_ptr<Input::AnalogDevice> Create(const Common::ParamPackage& params) override {
        int axisX = params.Get("axis_x", 0);
        int axisY = params.Get("axis_y", 0);
        return std::make_unique<Analog>(axisX, axisY);
    }
};

class MotionFactory final : public Input::Factory<Input::MotionDevice> {
public:
    std::unique_ptr<Input::MotionDevice> Create(const Common::ParamPackage& params) override {
        return std::make_unique<Motion>();
    }
};

class TouchFactory final : public Input::Factory<Input::TouchDevice> {
public:
    std::unique_ptr<Input::TouchDevice> Create(const Common::ParamPackage& params) override {
        int axisX = params.Get("axis_x", 0);
        int axisY = params.Get("axis_y", 0);
        int axisZ = params.Get("axis_z", 0);
        return std::make_unique<Touch>(axisX, axisY, axisZ);
    }
};

InputManager& InputManager::GetInstance() {
    static InputManager s_input_manager;
    return s_input_manager;
}

InputManager::InputManager() {
    mInputs.resize(N3DS_INPUT_COUNT + 1);
    Input::RegisterFactory<Input::ButtonDevice>(TOUCHSCREEN, std::make_shared<ButtonFactory>());
    Input::RegisterFactory<Input::AnalogDevice>(TOUCHSCREEN, std::make_shared<AnalogFactory>());
    Input::RegisterFactory<Input::MotionDevice>(TOUCHSCREEN, std::make_shared<MotionFactory>());
    Input::RegisterFactory<Input::TouchDevice>(TOUCHSCREEN, std::make_shared<TouchFactory>());
}

InputManager::~InputManager() {
    Input::UnregisterFactory<Input::ButtonDevice>(TOUCHSCREEN);
    Input::UnregisterFactory<Input::AnalogDevice>(TOUCHSCREEN);
    Input::UnregisterFactory<Input::MotionDevice>(TOUCHSCREEN);
    Input::UnregisterFactory<Input::TouchDevice>(TOUCHSCREEN);
}

void InputManager::InitProfile() {
    Settings::values.current_input_profile_index = 0;
    Settings::InputProfile profile;
    profile.name = TOUCHSCREEN;

    for (int i = 0; i < Settings::NativeButton::NumButtons; ++i) {
        profile.buttons[i] = fmt::format("button:{},engine:{}", i, TOUCHSCREEN);
    }

    profile.analogs[Settings::NativeAnalog::CirclePad] =
        fmt::format("axis_x:{},axis_y:{},engine:{}", N3DS_CPAD_X, N3DS_CPAD_Y, TOUCHSCREEN);
    profile.analogs[Settings::NativeAnalog::CStick] =
        fmt::format("axis_x:{},axis_y:{},engine:{}", N3DS_STICK_X, N3DS_STICK_Y, TOUCHSCREEN);

    profile.motion_device = fmt::format("engine:{}", TOUCHSCREEN);
    profile.touch_device = "engine:emu_window";
    profile.udp_input_address = "127.0.0.1";
    profile.udp_input_port = 26760;
    profile.udp_pad_index = 0;

    Settings::values.input_profiles.emplace_back(std::move(profile));
    Settings::LoadProfile(Settings::values.current_input_profile_index);
}

float InputManager::GetInput(int button) {
    if (button >= 0 && button < N3DS_INPUT_COUNT) {
        return mInputs[button];
    }
    return 0.0F;
}

bool InputManager::InputEvent(const std::string& dev, int button, float value) {
    if (button >= 0 && button < N3DS_INPUT_COUNT) {
        mInputs[button] = value;
        return true;
    }
    return false;
}
