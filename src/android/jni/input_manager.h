#pragma once

#include <string>
#include <vector>

class ButtonFactory;
class AnalogFactory;
class NDKMotionFactory;

class InputManager {
public:
    enum ButtonType {
        N3DS_BUTTON_A = 0,
        N3DS_BUTTON_B = 1,
        N3DS_BUTTON_X = 2,
        N3DS_BUTTON_Y = 3,
        N3DS_DPAD_UP = 4,
        N3DS_DPAD_DOWN = 5,
        N3DS_DPAD_LEFT = 6,
        N3DS_DPAD_RIGHT = 7,
        N3DS_BUTTON_L = 8,
        N3DS_BUTTON_R = 9,
        N3DS_BUTTON_START = 10,
        N3DS_BUTTON_SELECT = 11,
        N3DS_BUTTON_DEBUG = 12,
        N3DS_BUTTON_GPIO14 = 13,
        N3DS_BUTTON_ZL = 14,
        N3DS_BUTTON_ZR = 15,
        N3DS_BUTTON_HOME = 16,
        N3DS_CPAD_X = 17,
        N3DS_CPAD_Y = 18,
        N3DS_STICK_X = 19,
        N3DS_STICK_Y = 20,

        N3DS_INPUT_COUNT = 21,
    };

    static InputManager& GetInstance();

    void Init();

    void Shutdown();

    void InitProfile();

    bool InputEvent(int button, float value);
    bool KeyEvent(int button, float value);

    void BeginTilt(int x, int y);
    void Tilt(int x, int y);
    void EndTilt();

    void SetDisplayRotation(int rotation);

private:
    InputManager() = default;

    std::vector<int> mButtonKeys;
    std::vector<int> mAnalogKeys;
    std::vector<float> mAnalogValues;

    std::shared_ptr<ButtonFactory> mButtonFactory;
    std::shared_ptr<AnalogFactory> mAnalogFactory;
    std::shared_ptr<NDKMotionFactory> mMotionFactory;
};
