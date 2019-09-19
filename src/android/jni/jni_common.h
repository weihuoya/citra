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
void UpdateProgress(const std::string& name, u32 written, u32 total);
void NotifyGameShudown();
void ShowInputBoxDialog(int maxLength, const std::string& hint, const std::string& button0,
                        const std::string& button1, const std::string& button2);
void ShowMiiSelectorDialog(bool cancel, const std::string& title,
                           const std::vector<std::string>& miis);
void ShowMessageDialog(int type, const std::string& msg);
