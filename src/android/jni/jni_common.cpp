#include "jni_common.h"

std::atomic<int> NativeLibrary::current_display_rotation = 0;

static constexpr char* CLASS = "org/citra/emu/NativeLibrary";
static NativeLibrary::ImageLoadedHandler s_image_loaded_callback;

// jni for cubeb
JNIEnv* cubeb_get_jni_env_for_thread() {
    return JniHelper::GetEnvForThread();
}

jobject cubeb_jni_get_context_instance() {
    return NativeLibrary::GetEmulationContext();
}

void NativeLibrary::Initialize(JNIEnv* env) {
    // todo
}

void NativeLibrary::Shutdown(JNIEnv* env) {
    JniHelper::CallStaticMethod<void>(CLASS, "notifyGameShudown");
}

int NativeLibrary::SafOpen(const std::string& path, const std::string& mode) {
    return JniHelper::CallStaticMethod<int>(CLASS, "SafOpen", path, mode);
}

u64 NativeLibrary::SafLastModified(const std::string& path) {
    return JniHelper::CallStaticMethod<jlong>(CLASS, "SafLastModified", path);
}

int NativeLibrary::SafClose(int fd) {
    return JniHelper::CallStaticMethod<int>(CLASS, "SafClose", fd);
}

// NativeLibrary
jobject NativeLibrary::GetEmulationContext() {
    return JniHelper::CallStaticObjectMethod(CLASS, "getEmulationContext", "()Landroid/content/Context;");
}

int NativeLibrary::GetDisplayRotation() {
    return JniHelper::CallStaticMethod<int>(CLASS, "getDisplayRotation");
}

bool NativeLibrary::IsPortrait() {
    return current_display_rotation == 0 || current_display_rotation == 2;
}

bool NativeLibrary::CheckRecordPermission() {
    const std::string permission{"android.permission.RECORD_AUDIO"};
    return JniHelper::CallStaticMethod<bool>(CLASS, "checkPermission", permission);
}

void NativeLibrary::SaveImageToFile(const std::string& path, const std::vector<u8>& pixels,
                                    u32 width, u32 height) {
    jintArray array = JniHelper::Wrap(reinterpret_cast<const u32*>(pixels.data()), width * height);
    JniHelper::CallStaticMethod<void>(CLASS, "saveImageToFile", path, array, width, height);
    JniHelper::GetEnvForThread()->DeleteLocalRef(array);
}

void NativeLibrary::LoadImageFromFile(std::vector<u8>& pixels, u32& width, u32& height,
                                      const std::string& path) {
    s_image_loaded_callback = [&pixels, &width, &height](u32* data32, u32 w, u32 h) -> void {
        u32 size = w * h * 4;
        u8* data8 = reinterpret_cast<u8*>(data32);
        pixels.clear();
        pixels.insert(pixels.begin(), data8, data8 + size);
        width = w;
        height = h;
    };
    JniHelper::CallStaticMethod<void>(CLASS, "loadImageFromFile", path);
}

void NativeLibrary::UpdateProgress(const std::string& name, u64 progress, u64 total) {
    JniHelper::CallStaticMethod<void>(CLASS, "updateProgress", name, progress, total);
}

void NativeLibrary::ShowInputBoxDialog(int maxLength, const std::string& error,
                                       const std::string& hint, const std::string& button0,
                                       const std::string& button1, const std::string& button2) {
    JniHelper::CallStaticMethod<void>(CLASS, "showInputBoxDialog", maxLength, error, hint,
                                      button0, button1, button2);
}

void NativeLibrary::ShowMiiSelectorDialog(bool cancel, const std::string& title,
                                          const std::vector<std::string>& miis) {
    JniHelper::CallStaticMethod<void>(CLASS, "showInputBoxDialog", cancel, title, miis);
}

void NativeLibrary::ShowMessageDialog(int type, const std::string& msg) {
    JniHelper::CallStaticMethod<void>(CLASS, "showMessageDialog", type, msg);
}

void NativeLibrary::AddNetPlayMessage(int type, const std::string& msg) {
    JniHelper::CallStaticMethod<void>(CLASS, "addNetPlayMessage", type, msg);
}

void NativeLibrary::PickImage(u32 width, u32 height, ImageLoadedHandler handler) {
    s_image_loaded_callback = handler;
    JniHelper::CallStaticMethod<void>(CLASS, "pickImage", width, height);
}

int NativeLibrary::GetSafeInsetLeft() {
    return JniHelper::CallStaticMethod<int>(CLASS, "getSafeInsetLeft");
}

int NativeLibrary::GetSafeInsetTop() {
    return JniHelper::CallStaticMethod<int>(CLASS, "getSafeInsetTop");
}

int NativeLibrary::GetSafeInsetRight() {
    return JniHelper::CallStaticMethod<int>(CLASS, "getSafeInsetRight");
}

int NativeLibrary::GetSafeInsetBottom() {
    return JniHelper::CallStaticMethod<int>(CLASS, "getSafeInsetBottom");
}

float NativeLibrary::GetScaleDensity() {
    return JniHelper::CallStaticMethod<float>(CLASS, "getScaleDensity");
}

void NativeLibrary::HandleNFCScanning(bool isScanning) {
    JniHelper::CallStaticMethod<void>(CLASS, "handleNFCScanning", isScanning);
}

void NativeLibrary::ImageLoadedCallback(u32* pixels, u32 width, u32 height) {
    s_image_loaded_callback(pixels, width, height);
    s_image_loaded_callback = nullptr;
}
