#include "jni_common.h"

#include <android/log.h>

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

private:
    jclass mClazz;
    jmethodID mGetEmulationContext;
    jmethodID mSaveImageToFile;
};

static constexpr jint JNI_VERSION = JNI_VERSION_1_6;
static JavaVM* s_java_vm = nullptr;
static std::unique_ptr<NativeLibrary> s_native_library;

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
    return GetEnvForThread()->NewStringUTF(str.c_str());
}

jintArray ToJIntArray(const u32* buffer, size_t size) {
    JNIEnv* env = GetEnvForThread();
    jintArray array = env->NewIntArray(size);
    if (!array) {
        __android_log_print(ANDROID_LOG_INFO, "zhangwei", "ToJIntArray error");
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
    return s_native_library->GetEmulationContext();
}

void SaveImageToFile(const std::string& path, u32 width, u32 height, const u32* pixels) {
    jintArray array = ToJIntArray(pixels, width * height);
    s_native_library->SaveImageToFile(ToJString(path), width, height, array);
    GetEnvForThread()->DeleteLocalRef(array);
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
