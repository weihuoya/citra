#include "input_manager.h"

#include <tuple>
#include <vector>

#include <sys/system_properties.h>
#include <android/log.h>
#include <fmt/format.h>

#include "core/frontend/input.h"
#include "core/settings.h"
#include "input_common/main.h"
#include "input_common/motion_emu.h"

#include "config/main_settings.h"
#include "ndk_motion.h"

static int GetPropertyInteger(const char * name, int defaultValue) {
    int result = defaultValue;
#ifdef __ANDROID__
    char valueText[PROP_VALUE_MAX] = {0};
    if (__system_property_get(name, valueText) != 0) {
        result = atoi(valueText);
    }
#else
    (void) name;
#endif
    return result;
}

static int GetSdkVersion() {
    static int sCachedSdkVersion = -1;
#ifdef __ANDROID__
    if (sCachedSdkVersion == -1) {
        sCachedSdkVersion = GetPropertyInteger("ro.build.version.sdk", -1);
    }
#endif
    return sCachedSdkVersion;
}

class ButtonList {
private:
    // Analog Button
    class AnalogButton final : public Input::ButtonDevice {
    public:
        explicit AnalogButton(ButtonList* button_list, int id, float threshold, bool trigger_if_greater)
                : button_list(button_list), button_id(id), threshold(threshold), trigger_if_greater(trigger_if_greater) {}

        ~AnalogButton() override {
            button_list->DestroyButton(this);
        }

        bool GetStatus() const override {
            if (trigger_if_greater)
                return axis.load() > threshold;
            return axis.load() < threshold;
        }

        void SetStatus(float value) {
            axis.store(value);
        }

        int GetButtonId() const {
            return button_id;
        }

    private:
        ButtonList* button_list;
        std::atomic<float> axis{0.0f};
        int button_id;
        float threshold;
        bool trigger_if_greater;
    };

public:
    std::unique_ptr<Input::ButtonDevice> CreateButton(int button_id, float threshold, bool trigger_if_greater) {
        std::lock_guard<std::mutex> guard(mutex);
        std::unique_ptr<AnalogButton> button =
                std::make_unique<AnalogButton>(this, button_id, threshold, trigger_if_greater);
        buttons.push_back(button.get());
        return std::move(button);
    }

    void DestroyButton(AnalogButton* target) {
        std::lock_guard<std::mutex> guard(mutex);
        auto iter = std::remove_if(buttons.begin(), buttons.end(), [target](const AnalogButton* button) {
            return button == target;
        });
        buttons.erase(iter, buttons.end());
    }

    bool ChangeButtonValue(int button_id, float value) {
        std::lock_guard<std::mutex> guard(mutex);
        bool is_found = false;
        for (auto& button : buttons) {
            if (button->GetButtonId() == button_id) {
                button->SetStatus(value);
                is_found = true;
            }
        }
        // If we don't find the button don't consume the button press event
        return is_found;
    }

private:
    std::mutex mutex;
    std::vector<AnalogButton*> buttons;
};

class AnalogList {
private:
    // Joystick Handler
    class Joystick final : public Input::AnalogDevice {
    public:
        explicit Joystick(AnalogList * button_list, int id) : button_list(button_list), button_id(id) {}

        ~Joystick() override {
            button_list->DestroyButton(this);
        }

        std::tuple<float, float> GetStatus() const override {
            return std::make_tuple(x_axis.load(), y_axis.load());
        }

        void SetStatus(float x, float y) {
            // Clamp joystick movement to supported minimum and maximum
            // Citra uses an inverted y axis sent by the frontend
            x = std::clamp(x, -1.0F, 1.0F);
            y = std::clamp(-y, -1.0F, 1.0F);

            // Clamp the input to a circle (while touch input is already clamped in the frontend, gamepad is
            // unknown)
            float r = x * x + y * y;
            if (r > 1.0f) {
                r = std::sqrt(r);
                x /= r;
                y /= r;
            }

            x_axis.store(x);
            y_axis.store(y);
        }

        int GetButtonId() const {
            return button_id;
        }

    private:
        AnalogList * button_list;
        int button_id;
        std::atomic<float> x_axis{0.0f};
        std::atomic<float> y_axis{0.0f};
    };

public:
    std::unique_ptr<Input::AnalogDevice> CreateButton(int button_id) {
        std::lock_guard<std::mutex> guard(mutex);
        std::unique_ptr<Joystick> analog = std::make_unique<Joystick>(this, button_id);
        buttons.push_back(analog.get());
        return std::move(analog);
    }

    void DestroyButton(Joystick* target) {
        std::lock_guard<std::mutex> guard(mutex);
        auto iter = std::remove_if(buttons.begin(), buttons.end(), [target](const Joystick* button) {
            return button == target;
        });
        buttons.erase(iter, buttons.end());
    }

    bool ChangeJoystickStatus(int button_id, float x, float y) {
        std::lock_guard<std::mutex> guard(mutex);
        bool is_found = false;
        for (auto& button : buttons) {
            if (button->GetButtonId() == button_id) {
                button->SetStatus(x, y);
                is_found = true;
            }
        }
        return is_found;
    }

private:
    std::mutex mutex;
    std::vector<Joystick*> buttons;
};

/**
 * A button device factory representing a gamepad. It receives input events and forward them
 * to all button devices it created.
 */
class ButtonFactory final : public Input::Factory<Input::ButtonDevice> {
public:
    ButtonFactory() : button_list{std::make_shared<ButtonList>()} {}

    /**
     * Creates a button device from a gamepad button
     * @param params contains parameters for creating the device:
     *     - "code": the code of the key to bind with the button
     */
    std::unique_ptr<Input::ButtonDevice> Create(const Common::ParamPackage& params) override {
        int button_id = params.Get("code", 0);
        float threshold = params.Get("threshold", 0.5f);
        bool trigger_if_greater = true;
        if (params.Get("dir", "+") == "-") {
            trigger_if_greater = false;
            if (threshold > 0) {
                threshold = -threshold;
            }
        }
        return button_list->CreateButton(button_id, threshold, trigger_if_greater);
    }

    /**
     * Sets the status of all buttons bound with the key to released
     * @param axis_id the code of the axis
     * @param axis_val the value of the axis
     * @return whether the key event is consumed or not
     */
    bool ButtonEvent(int button_id, float value) {
        return button_list->ChangeButtonValue(button_id, value);
    }

private:
    std::shared_ptr<ButtonList> button_list;
};

/**
 * An analog device factory representing a gamepad(virtual or physical). It receives input events
 * and forward them to all analog devices it created.
 */
class AnalogFactory final : public Input::Factory<Input::AnalogDevice> {
public:
    AnalogFactory() : analog_list{std::make_shared<AnalogList>()} {}

    /**
     * Creates an analog device from the gamepad joystick
     * @param params contains parameters for creating the device:
     *     - "code": the code of the key to bind with the button
     */
    std::unique_ptr<Input::AnalogDevice> Create(const Common::ParamPackage& params) override {
        int button_id = params.Get("code", 0);
        return analog_list->CreateButton(button_id);
    }

    /**
     * Sets the status of all buttons bound with the key to pressed
     * @param key_code the code of the analog stick
     * @param x the x-axis value of the analog stick
     * @param y the y-axis value of the analog stick
     */
    bool AnalogEvent(int analog_id, float x, float y) {
        return analog_list->ChangeJoystickStatus(analog_id, x, y);
    }

private:
    std::shared_ptr<AnalogList> analog_list;
};

/**
 * Input Manager
 */
InputManager& InputManager::GetInstance() {
    static InputManager s_input_manager;
    return s_input_manager;
}

void InputManager::Init() {
    mButtonFactory = std::make_shared<ButtonFactory>();
    mAnalogFactory = std::make_shared<AnalogFactory>();
    Input::RegisterFactory<Input::ButtonDevice>("android", mButtonFactory);
    Input::RegisterFactory<Input::AnalogDevice>("android", mAnalogFactory);
    if (GetSdkVersion() >= 26) {
        mMotionFactory = std::make_shared<NDKMotionFactory>();
        Input::RegisterFactory<Input::MotionDevice>("android", mMotionFactory);
    }

    InitProfile();
}

void InputManager::Shutdown() {
    Input::UnregisterFactory<Input::ButtonDevice>("android");
    Input::UnregisterFactory<Input::AnalogDevice>("android");
    mButtonFactory.reset();
    mAnalogFactory.reset();
    if (mMotionFactory) {
        Input::UnregisterFactory<Input::MotionDevice>("android");
        mMotionFactory.reset();
    }
}

void InputManager::InitProfile() {
    Settings::values.current_input_profile_index = 0;
    Settings::InputProfile profile;
    profile.name = "android";

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
        param.Set("engine", "android");
        profile.buttons[i] = param.Serialize();
        mButtonKeys.push_back(keyCode);
    }

    std::array<std::string, 4> AnalogConfigs = {
        Config::Get(Config::CIRCLE_PAD_X),   Config::Get(Config::CIRCLE_PAD_Y),
        Config::Get(Config::C_STICK_X), Config::Get(Config::C_STICK_Y),
    };

    mAnalogKeys.clear();
    for (int i = 0; i < AnalogConfigs.size(); ++i) {
        const std::string& config = AnalogConfigs[i];
        Common::ParamPackage param(config);
        int keyCode = param.Get("code", i);
        mAnalogKeys.push_back(keyCode);
    }

    mAnalogValues.clear();
    mAnalogValues.resize(4);
    profile.analogs[Settings::NativeAnalog::CirclePad] = "engine:android,code:0";
    profile.analogs[Settings::NativeAnalog::CStick] = "engine:android,code:1";

    profile.motion_device = "engine:android,update_period:100,sensitivity:0.01,tilt_clamp:90.0";
    profile.touch_device = "engine:emu_window";
    profile.udp_input_address = "127.0.0.1";
    profile.udp_input_port = 26760;
    profile.udp_pad_index = 0;

    Settings::values.input_profiles.clear();
    Settings::values.input_profiles.emplace_back(std::move(profile));
    Settings::LoadProfile(Settings::values.current_input_profile_index);
}

bool InputManager::InputEvent(int button, float value) {
    bool handled = true;
    if (button == N3DS_CPAD_X) {
        mAnalogValues[0] = value;
        mAnalogFactory->AnalogEvent(0, mAnalogValues[0], mAnalogValues[1]);
    } else if (button == N3DS_CPAD_Y) {
        mAnalogValues[1] = value;
        mAnalogFactory->AnalogEvent(0, mAnalogValues[0], mAnalogValues[1]);
    } else if (button == N3DS_STICK_X) {
        mAnalogValues[2] = value;
        mAnalogFactory->AnalogEvent(1, mAnalogValues[2], mAnalogValues[3]);
    } else if (button == N3DS_STICK_Y) {
        mAnalogValues[3] = value;
        mAnalogFactory->AnalogEvent(1, mAnalogValues[2], mAnalogValues[3]);
    } else {
        if (button == N3DS_DPAD_LEFT || button == N3DS_DPAD_UP) {
            value = -value;
        }
        handled = KeyEvent(mButtonKeys[button], value);
    }
    return handled;
}

bool InputManager::KeyEvent(int button, float value) {
    // buttons
    if (mButtonFactory->ButtonEvent(button, value)) {
        return true;
    }

    // analogs
    for (int i = 0; i < mAnalogKeys.size(); ++i) {
        if (mAnalogKeys[i] == button) {
            int code = i >> 1;
            mAnalogValues[i] = value;
            mAnalogFactory->AnalogEvent(code, mAnalogValues[code * 2], mAnalogValues[code * 2 + 1]);
            return true;
        }
    }

    return false;
}

void InputManager::SetDisplayRotation(int rotation) {
    ndkmotion_display_rotation = rotation;
}