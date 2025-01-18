#pragma once

#include <atomic>

#include "jni_helper.h"

class NativeLibrary {
public:
    using ImageLoadedHandler = std::function<void(u32* pixels, u32 width, u32 height)>;

    static void Initialize(JNIEnv* env);
    static void Shutdown(JNIEnv* env);

    static int SafOpen(const std::string& path, const std::string& mode);
    static u64 SafLastModified(const std::string& path);
    static int SafClose(int fd);

    static jobject GetEmulationContext();
    static int GetDisplayRotation();
    static bool IsPortrait();
    static bool CheckRecordPermission();
    static void SaveImageToFile(const std::string& path, const std::vector<u8>& pixels, u32 width,
                                u32 height);
    static void LoadImageFromFile(std::vector<u8>& pixels, u32& width, u32& height,
                                  const std::string& path);
    static void UpdateProgress(const std::string& name, u64 progress, u64 total);
    static void ShowInputBoxDialog(int maxLength, const std::string& error, const std::string& hint,
                                   const std::string& button0, const std::string& button1,
                                   const std::string& button2);
    static void ShowMiiSelectorDialog(bool cancel, const std::string& title,
                                      const std::vector<std::string>& miis);
    static void ShowMessageDialog(int type, const std::string& msg);
    static void AddNetPlayMessage(int type, const std::string& msg);
    static void PickImage(u32 width, u32 height, ImageLoadedHandler handler);
    static int GetSafeInsetLeft();
    static int GetSafeInsetTop();
    static int GetSafeInsetRight();
    static int GetSafeInsetBottom();
    static float GetScaleDensity();
    static void HandleNFCScanning(bool isScanning);
    static void ImageLoadedCallback(u32* jpixels, u32 width, u32 height);

    static std::atomic<int> current_display_rotation;
};
