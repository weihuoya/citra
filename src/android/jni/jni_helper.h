#pragma once

#include <string>
#include <vector>
#include <jni.h>

#include "common/common_types.h"

class JniHelper {
public:
    static void OnLoad();
    static void OnUnload();

    static JNIEnv* GetEnvForThread();

    static jobject RectObject(int left, int top, int right, int bottom);

    static jstring Wrap(std::string_view str) {
        return GetEnvForThread()->NewStringUTF(str.data());
    }

    static jbyteArray Wrap(const u8* data, std::size_t size) {
        JNIEnv* env = GetEnvForThread();
        std::vector<jobject> localRefs;
        return JniWrap(env, localRefs, data, size);
    }

    static jintArray Wrap(const u32* data, std::size_t size) {
        JNIEnv* env = GetEnvForThread();
        std::vector<jobject> localRefs;
        return JniWrap(env, localRefs, data, size);
    }

    static jintArray Wrap(const std::vector<u32>& value) {
        return Wrap(value.data(), value.size());
    }

    static jobjectArray Wrap(const std::vector<std::string>& value) {
        JNIEnv* env = GetEnvForThread();
        std::vector<jobject> localRefs;
        return JniWrap(env, localRefs, value);
    }

    static std::string Unwrap(jstring value) {
        return UnwrapString(GetEnvForThread(), value);
    }

    template <typename R>
    static R Unwrap(jobjectArray array) {
        return JniUnwrapObjectArray<R>::Unwrap(GetEnvForThread(), array);
    }

    template <typename R, typename... Ts>
    static R CallStaticMethod(const char* className, const char* methodName, Ts... xs) {
        return JniStaticMethod<R, Ts...>::Call(className, methodName, xs...);
    }

    template <typename... Ts>
    static jobject CallStaticObjectMethod(const char* className, const char* methodName,
                                          const char* signature, Ts... xs) {
        jobject ret;
        std::vector<jobject> localRefs;
        JNIEnv* env = GetEnvForThread();
        jclass clazz = FindClass(className);
        jmethodID method = env->GetStaticMethodID(clazz, methodName, signature);
        ret = env->CallStaticObjectMethod(clazz, method, JniWrap(env, localRefs, xs)...);
        DeleteLocalRefs(env, localRefs);
        return ret;
    }

private:
    static jclass FindClass(const char* className);

    /// methods
    template <typename R, typename... Ts>
    struct JniStaticMethod {
        static R Call(const char* className, const char* methodName, Ts... xs);
    };

    template <typename... Ts>
    struct JniStaticMethod<void, Ts...> {
        static void Call(const char* className, const char* methodName, Ts... xs) {
            std::vector<jobject> localRefs;
            JNIEnv* env = GetEnvForThread();
            std::string signature = "(" + JniSignature(xs...) + ")V";
            jclass clazz = FindClass(className);
            jmethodID method = env->GetStaticMethodID(clazz, methodName, signature.data());
            env->CallStaticVoidMethod(clazz, method, JniWrap(env, localRefs, xs)...);
            DeleteLocalRefs(env, localRefs);
        }
    };

    template <typename... Ts>
    struct JniStaticMethod<bool, Ts...> {
        static bool Call(const char* className, const char* methodName, Ts... xs) {
            jboolean ret;
            std::vector<jobject> localRefs;
            JNIEnv* env = GetEnvForThread();
            std::string signature = "(" + JniSignature(xs...) + ")Z";
            jclass clazz = FindClass(className);
            jmethodID method = env->GetStaticMethodID(clazz, methodName, signature.data());
            ret = env->CallStaticBooleanMethod(clazz, method, JniWrap(env, localRefs, xs)...);
            DeleteLocalRefs(env, localRefs);
            return (ret == JNI_TRUE);
        }
    };

    template <typename... Ts>
    struct JniStaticMethod<jint, Ts...> {
        static jint Call(const char* className, const char* methodName, Ts... xs) {
            jint ret;
            std::vector<jobject> localRefs;
            JNIEnv* env = GetEnvForThread();
            std::string signature = "(" + JniSignature(xs...) + ")I";
            jclass clazz = FindClass(className);
            jmethodID method = env->GetStaticMethodID(clazz, methodName, signature.data());
            ret = env->CallStaticIntMethod(clazz, method, JniWrap(env, localRefs, xs)...);
            DeleteLocalRefs(env, localRefs);
            return ret;
        }
    };

    template <typename... Ts>
    struct JniStaticMethod<jlong, Ts...> {
        static jlong Call(const char* className, const char* methodName, Ts... xs) {
            jlong ret;
            std::vector<jobject> localRefs;
            JNIEnv* env = GetEnvForThread();
            std::string signature = "(" + JniSignature(xs...) + ")J";
            jclass clazz = FindClass(className);
            jmethodID method = env->GetStaticMethodID(clazz, methodName, signature.data());
            ret = env->CallStaticLongMethod(clazz, method, JniWrap(env, localRefs, xs)...);
            DeleteLocalRefs(env, localRefs);
            return ret;
        }
    };

    template <typename... Ts>
    struct JniStaticMethod<jfloat, Ts...> {
        static jfloat Call(const char* className, const char* methodName, Ts... xs) {
            jfloat ret;
            std::vector<jobject> localRefs;
            JNIEnv* env = GetEnvForThread();
            std::string signature = "(" + JniSignature(xs...) + ")F";
            jclass clazz = FindClass(className);
            jmethodID method = env->GetStaticMethodID(clazz, methodName, signature.data());
            ret = env->CallStaticFloatMethod(clazz, method, JniWrap(env, localRefs, xs)...);
            DeleteLocalRefs(env, localRefs);
            return ret;
        }
    };

    /// jni unwrap
    static std::string UnwrapString(JNIEnv* env, jstring value) {
        std::string result;
        const char* str = env->GetStringUTFChars(value, nullptr);
        result = str;
        env->ReleaseStringUTFChars(value, str);
        // env->DeleteLocalRef(value);
        return result;
    }

    template <typename R>
    struct JniUnwrapObjectArray {
        static R Unwrap(JNIEnv* env, jobjectArray array);
    };

    template <>
    struct JniUnwrapObjectArray<std::vector<std::string>> {
        static std::vector<std::string> Unwrap(JNIEnv* env, jobjectArray array) {
            const jsize size = env->GetArrayLength(array);
            std::vector<std::string> result;
            for (jsize i = 0; i < size; ++i) {
                jstring value = (jstring)env->GetObjectArrayElement(array, i);
                result.push_back(UnwrapString(env, value));
            }
            env->DeleteLocalRef(array);
            return result;
        }
    };

    /// jni wrap
    static jobjectArray JniWrap(JNIEnv* env, std::vector<jobject>& localRefs,
                                const std::vector<std::string>& value) {
        jclass clazz = FindClass("java/lang/String");
        jstring args = env->NewStringUTF("");
        jobjectArray array = env->NewObjectArray(value.size(), clazz, args);
        localRefs.push_back(args);
        for (int i = 0; i < value.size(); ++i) {
            jstring element = env->NewStringUTF(value[i].data());
            env->SetObjectArrayElement(array, i, element);
            localRefs.push_back(element);
        }
        localRefs.push_back(array);
        return array;
    }
    static jbyteArray JniWrap(JNIEnv* env, std::vector<jobject>& localRefs, const u8* data,
                              std::size_t size) {
        jbyteArray array = env->NewByteArray(size);
        env->SetByteArrayRegion(array, 0, size, reinterpret_cast<const jbyte*>(data));
        localRefs.push_back(array);
        return array;
    }
    static jintArray JniWrap(JNIEnv* env, std::vector<jobject>& localRefs, const u32* data,
                             std::size_t size) {
        jintArray array = env->NewIntArray(size);
        env->SetIntArrayRegion(array, 0, size, reinterpret_cast<const jint*>(data));
        localRefs.push_back(array);
        return array;
    }
    static jstring JniWrap(JNIEnv* env, std::vector<jobject>& localRefs, const std::string& value) {
        jstring str = env->NewStringUTF(value.data());
        localRefs.push_back(str);
        return str;
    }
    static jstring JniWrap(JNIEnv* env, std::vector<jobject>& localRefs, const std::string_view& value) {
        jstring str = env->NewStringUTF(value.data());
        localRefs.push_back(str);
        return str;
    }
    static jint JniWrap(JNIEnv*, std::vector<jobject>&, int32_t value) {
        return static_cast<jint>(value);
    }
    static jlong JniWrap(JNIEnv*, std::vector<jobject>&, int64_t value) {
        return static_cast<jlong>(value);
    }
    static jfloat JniWrap(JNIEnv*, std::vector<jobject>&, float value) {
        return static_cast<jfloat>(value);
    }
    static jdouble JniWrap(JNIEnv*, std::vector<jobject>&, double value) {
        return static_cast<jdouble>(value);
    }
    static jboolean JniWrap(JNIEnv*, std::vector<jobject>&, bool value) {
        return static_cast<jboolean>(value);
    }
    static jbyte JniWrap(JNIEnv*, std::vector<jobject>&, int8_t value) {
        return static_cast<jbyte>(value);
    }
    static jchar JniWrap(JNIEnv*, std::vector<jobject>&, uint8_t value) {
        return static_cast<jchar>(value);
    }
    static jshort JniWrap(JNIEnv*, std::vector<jobject>&, int16_t value) {
        return static_cast<jshort>(value);
    }

    template <typename T>
    static T JniWrap(JNIEnv* env, std::vector<jobject>& localRefs, T x) {
        return x;
    }

    /// release refs
    static void DeleteLocalRefs(JNIEnv* env, std::vector<jobject>& localRefs) {
        for (const auto& ref : localRefs) {
            env->DeleteLocalRef(ref);
        }
        localRefs.clear();
    }

    /// signature
    static std::string JniSignature() {
        return "";
    }

    static std::string JniSignature(bool) {
        return "Z";
    }

    static std::string JniSignature(jchar) {
        return "C";
    }

    static std::string JniSignature(jshort) {
        return "S";
    }

    static std::string JniSignature(jint) {
        return "I";
    }

    static std::string JniSignature(uint32_t) {
        return "I";
    }

    static std::string JniSignature(jlong) {
        return "J";
    }

    static std::string JniSignature(uint64_t) {
        return "J";
    }

    static std::string JniSignature(jfloat) {
        return "F";
    }

    static std::string JniSignature(jdouble) {
        return "D";
    }

    static std::string JniSignature(const char*) {
        return "Ljava/lang/String;";
    }

    static std::string JniSignature(const std::string&) {
        return "Ljava/lang/String;";
    }

    static std::string JniSignature(const std::string_view&) {
        return "Ljava/lang/String;";
    }

    static std::string JniSignature(const std::vector<std::string>&) {
        return "[Ljava/lang/String;";
    }

    static std::string JniSignature(jintArray) {
        return "[I";
    }

    template <typename T>
    static std::string JniSignature(T x) {
        // This template should never be instantiated
        static_assert(sizeof(x) == 0, "Unsupported argument type");
        return "";
    }

    template <typename T, typename... Ts>
    static std::string JniSignature(T x, Ts... xs) {
        return JniSignature(x) + JniSignature(xs...);
    }
};