#pragma once

#include <string>
#include <vector>
#include <jni.h>

#include "common/common_types.h"

std::string GetJString(jstring jstr);
jstring ToJString(const std::string& str);
jintArray ToJIntArray(const std::vector<u32>& buffer);
JNIEnv* GetEnvForThread();
jobject GetEmulationContext();
void SaveImageToFile(const std::string& path, u32 width, u32 height,
                     const std::vector<u32>& pixels);
