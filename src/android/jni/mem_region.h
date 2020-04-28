#pragma once

#include "jni_common.h"

jintArray searchMemoryRegion(u32 start_addr, u32 stop_addr, u32 value_type, u32 search_type, u32 scan_type, u32 value);
jintArray getSearchResults();
void resetSearchResults();
