#include <unordered_map>

#include "jni_helper.h"
#include "multiplayer.h"

static constexpr jint JNI_VERSION = JNI_VERSION_1_6;
static JavaVM* sJavaVM = nullptr;
static std::unordered_map<std::string, jclass> sClassMap;

// The FindClass method should be called from Java thread only.
// FindClass's implementation is looking for a ClassLoader by traversing the current call-stack.
// If you are trying to call the FindClass from a native thread, there is no ClassLoader to look for.

JNIEnv* JniHelper::GetEnvForThread() {
    thread_local static struct OwnedEnv {
        OwnedEnv() {
            status = sJavaVM->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION);
            if (status == JNI_EDETACHED)
                sJavaVM->AttachCurrentThread(&env, nullptr);
        }

        ~OwnedEnv() {
            if (status == JNI_EDETACHED)
                sJavaVM->DetachCurrentThread();
        }

        int status;
        JNIEnv* env = nullptr;
    } owned;
    return owned.env;
}

void JniHelper::OnLoad() {
    JNIEnv* env = GetEnvForThread();
    const char* clazz = "java/lang/String";
    sClassMap.emplace(clazz, (jclass)env->NewGlobalRef(env->FindClass(clazz)));

    clazz = "android/graphics/Rect";
    sClassMap.emplace(clazz, (jclass)env->NewGlobalRef(env->FindClass(clazz)));

    clazz = "org/citra/emu/NativeLibrary";
    sClassMap.emplace(clazz, (jclass)env->NewGlobalRef(env->FindClass(clazz)));
}

void JniHelper::OnUnload() {
    JNIEnv* env = GetEnvForThread();
    for (auto& pair : sClassMap) {
        env->DeleteGlobalRef(pair.second);
    }
}

jobject JniHelper::RectObject(int left, int top, int right, int bottom) {
    JNIEnv* env = GetEnvForThread();
    jclass clazz = FindClass("android/graphics/Rect");
    jmethodID method = env->GetMethodID(clazz, "<init>", "(IIII)V");
    jobject obj = env->NewObject(clazz, method, left, top, right, bottom);
    return obj;
}

jclass JniHelper::FindClass(const char* className) {
    return sClassMap[className];
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION) != JNI_OK)
        return JNI_ERR;

    sJavaVM = vm;
    JniHelper::OnLoad();
    NetworkInit();

    return JNI_VERSION;
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void* reserved) {
    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION) != JNI_OK)
        return;

    JniHelper::OnUnload();
    NetworkShutdown();
}

#ifdef __cplusplus
}
#endif