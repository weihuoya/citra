#pragma once

#include <string>
#include <vector>
#include <jni.h>

#include "common/common_types.h"

std::string GetJString(jstring jstr);
jstring ToJString(const std::string& str);
jintArray ToJIntArray(const u32* buffer, std::size_t size);
jobjectArray ToJStringArray(const std::vector<std::string>& strs);
std::vector<std::string> JStringArrayToVector(jobjectArray array);

JNIEnv* GetEnvForThread();
jobject GetEmulationContext();

extern std::function<void(u32* pixels, u32 width, u32 height)> g_image_loading_callback;
void SaveImageToFile(const std::string& path, const std::vector<u8>& pixels, u32 width, u32 height);
void LoadImageFromFile(const std::string& path, std::vector<u8>& pixels, u32& width, u32& height);
void UpdateProgress(const std::string& name, u32 written, u32 total);
void NotifyGameShudown();
void ShowInputBoxDialog(int maxLength, const std::string& error, const std::string& hint, const std::string& button0,
                        const std::string& button1, const std::string& button2);
void ShowMiiSelectorDialog(bool cancel, const std::string& title,
                           const std::vector<std::string>& miis);
void ShowMessageDialog(int type, const std::string& msg);
void PickImage(u32 width, u32 height);
