#include <android/log.h>
#include "jni_common.h"

class NativeLibrary {
public:
    NativeLibrary() {
        JNIEnv* env = GetEnvForThread();
        jclass clazz = env->FindClass("org/citra/emu/NativeLibrary");
        mClazz = reinterpret_cast<jclass>(env->NewGlobalRef(clazz));
        mGetEmulationContext =
            env->GetStaticMethodID(mClazz, "getEmulationContext", "()Landroid/content/Context;");
        mSaveImageToFile =
            env->GetStaticMethodID(mClazz, "saveImageToFile", "(Ljava/lang/String;II[I)V");
        mLoadImageFromFile =
            env->GetStaticMethodID(mClazz, "loadImageFromFile", "(Ljava/lang/String;)V");
        mUpdateProgress =
            env->GetStaticMethodID(mClazz, "updateProgress", "(Ljava/lang/String;II)V");
        mNotifyGameShudown = env->GetStaticMethodID(mClazz, "notifyGameShudown", "()V");
        mShowInputBoxDialog = env->GetStaticMethodID(
            mClazz, "showInputBoxDialog",
            "(ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
        mShowMessageDialog =
            env->GetStaticMethodID(mClazz, "showMessageDialog", "(ILjava/lang/String;)V");
        mShowMiiSelectorDialog = env->GetStaticMethodID(
            mClazz, "showMiiSelectorDialog", "(ZLjava/lang/String;[Ljava/lang/String;)V");
        mPickImage = env->GetStaticMethodID(mClazz, "pickImage", "(II)V");
        mHandleNFCScanning = env->GetStaticMethodID(mClazz, "handleNFCScanning", "(Z)V");
    }

    ~NativeLibrary() {
        GetEnvForThread()->DeleteGlobalRef(mClazz);
    }

    jobject GetEmulationContext() {
        return GetEnvForThread()->CallStaticObjectMethod(mClazz, mGetEmulationContext);
    }

    void SaveImageToFile(jstring path, u32 width, u32 height, jintArray pixels) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mSaveImageToFile, path, width, height,
                                                pixels);
    }

    void LoadImageFromFile(jstring path) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mLoadImageFromFile, path);
    }

    void UpdateProgress(jstring name, u32 written, u32 total) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mUpdateProgress, name, written, total);
    }

    void ShowInputBoxDialog(jint maxLength, jstring error, jstring hint, jstring button0, jstring button1,
                            jstring button2) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mShowInputBoxDialog, maxLength, error, hint,
                                                button0, button1, button2);
    }

    void ShowMiiSelectorDialog(jboolean cancel, jstring title, jobjectArray miis) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mShowMiiSelectorDialog, cancel, title,
                                                miis);
    }

    void ShowMessageDialog(jint type, jstring msg) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mShowMessageDialog, type, msg);
    }

    void NotifyGameShudown() {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mNotifyGameShudown);
    }

    void PickImage(jint width, jint height) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mPickImage, width, height);
    };

    void HandleNFCScanning(jboolean isScanning) {
        GetEnvForThread()->CallStaticVoidMethod(mClazz, mHandleNFCScanning, isScanning);
    };

private:
    jclass mClazz;
    jmethodID mGetEmulationContext;
    jmethodID mSaveImageToFile;
    jmethodID mLoadImageFromFile;
    jmethodID mUpdateProgress;
    jmethodID mNotifyGameShudown;
    jmethodID mShowInputBoxDialog;
    jmethodID mShowMessageDialog;
    jmethodID mShowMiiSelectorDialog;
    jmethodID mHandleNFCScanning;
    jmethodID mPickImage;
};

static constexpr jint JNI_VERSION = JNI_VERSION_1_6;
static JavaVM* s_java_vm = nullptr;
static std::unique_ptr<NativeLibrary> s_native_library;
std::function<void(u32* pixels, u32 width, u32 height)> g_image_loading_callback;

std::string GetJString(jstring jstr) {
    std::string result;
    if (!jstr)
        return result;

    JNIEnv* env = GetEnvForThread();
    const char* s = env->GetStringUTFChars(jstr, nullptr);
    result = s;
    env->ReleaseStringUTFChars(jstr, s);
    env->DeleteLocalRef(jstr);
    return result;
}

jstring ToJString(const std::string& str) {
    jstring jstr = GetEnvForThread()->NewStringUTF(str.c_str());
    return jstr;
}

jintArray ToJIntArray(const u32* buffer, size_t size) {
    JNIEnv* env = GetEnvForThread();
    jintArray array = env->NewIntArray(size);
    if (!array) {
        __android_log_print(ANDROID_LOG_INFO, "citra", "ToJIntArray error");
        return nullptr;
    }
    env->SetIntArrayRegion(array, 0, size, reinterpret_cast<const jint*>(buffer));
    return array;
}

jobjectArray ToJStringArray(const std::vector<std::string>& strs) {
    JNIEnv* env = GetEnvForThread();
    jobjectArray array =
        env->NewObjectArray(strs.size(), env->FindClass("java/lang/String"), env->NewStringUTF(""));
    for (int i = 0; i < strs.size(); ++i) {
        env->SetObjectArrayElement(array, i, ToJString(strs[i]));
    }
    return array;
}

std::vector<std::string> JStringArrayToVector(jobjectArray array) {
    JNIEnv* env = GetEnvForThread();
    const jsize size = env->GetArrayLength(array);
    std::vector<std::string> result;
    for (jsize i = 0; i < size; ++i)
        result.push_back(GetJString((jstring)env->GetObjectArrayElement(array, i)));
    return result;
}

jobject GetEmulationContext() {
    jobject context = s_native_library->GetEmulationContext();
    return context;
}

void SaveImageToFile(const std::string& path, const std::vector<u8>& pixels, u32 width,
                     u32 height) {
    jintArray array = ToJIntArray(reinterpret_cast<const u32*>(pixels.data()), width * height);
    s_native_library->SaveImageToFile(ToJString(path), width, height, array);
    GetEnvForThread()->DeleteLocalRef(array);
}

void LoadImageFromFile(const std::string& path, std::vector<u8>& pixels, u32& width, u32& height) {
    g_image_loading_callback = [&pixels, &width, &height](u32* data32, u32 w, u32 h) -> void {
        u32 size = w * h * 4;
        u8* data8 = reinterpret_cast<u8*>(data32);
        pixels.insert(pixels.begin(), data8, data8 + size);
        width = w;
        height = h;
    };
    s_native_library->LoadImageFromFile(ToJString(path));
}

void UpdateProgress(const std::string& name, u32 written, u32 total) {
    s_native_library->UpdateProgress(ToJString(name), written, total);
}

void NotifyGameShudown() {
    s_native_library->NotifyGameShudown();
}

void PickImage(u32 width, u32 height) {
    s_native_library->PickImage(width, height);
}

void HandleNFCScanning(bool isScanning) {
    s_native_library->HandleNFCScanning(isScanning);
}

void ShowInputBoxDialog(int maxLength, const std::string& error, const std::string& hint, const std::string& button0,
                        const std::string& button1, const std::string& button2) {
    s_native_library->ShowInputBoxDialog(maxLength, ToJString(error), ToJString(hint), ToJString(button0),
                                         ToJString(button1), ToJString(button2));
}

void ShowMiiSelectorDialog(bool cancel, const std::string& title,
                           const std::vector<std::string>& miis) {
    s_native_library->ShowMiiSelectorDialog(cancel, ToJString(title), ToJStringArray(miis));
}

void ShowMessageDialog(int type, const std::string& msg) {
    s_native_library->ShowMessageDialog(type, ToJString(msg));
}

JNIEnv* GetEnvForThread() {
    thread_local static struct OwnedEnv {
        OwnedEnv() {
            status = s_java_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION);
            if (status == JNI_EDETACHED)
                s_java_vm->AttachCurrentThread(&env, nullptr);
        }

        ~OwnedEnv() {
            if (status == JNI_EDETACHED)
                s_java_vm->DetachCurrentThread();
        }

        int status;
        JNIEnv* env = nullptr;
    } owned;
    return owned.env;
}

// jni for cubeb
JNIEnv* cubeb_get_jni_env_for_thread() {
    return GetEnvForThread();
}

jobject cubeb_jni_get_context_instance() {
    return GetEmulationContext();
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    s_java_vm = vm;

    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION) != JNI_OK)
        return JNI_ERR;

    s_native_library = std::make_unique<NativeLibrary>();

    return JNI_VERSION;
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void* reserved) {
    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION) != JNI_OK)
        return;

    s_native_library.reset();
}

#ifdef __cplusplus
}
#endif
