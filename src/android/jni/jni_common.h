#pragma once

#include <string>
#include <vector>
#include <jni.h>

#include "common/common_types.h"

std::string GetJString(jstring jstr);
jstring ToJString(const std::string& str);
jintArray ToJIntArray(const u32* buffer, size_t size);
jobjectArray ToJStringArray(const std::vector<std::string>& strs);
std::vector<std::string> JStringArrayToVector(jobjectArray array);

JNIEnv* GetEnvForThread();
jobject GetEmulationContext();

void SaveImageToFile(const std::string& path, u32 width, u32 height, const u32* pixels);
