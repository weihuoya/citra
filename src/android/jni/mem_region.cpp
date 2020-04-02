#include <android/log.h>
#include "mem_region.h"

#include "jni_common.h"
#include "common/common_types.h"
#include "core/core.h"
#include "core/memory.h"

// search type
static const u32 SEARCH_TYPE_SPECIFIED_VALUE = 0;
static const u32 SEARCH_TYPE_UNKNOWN_SEARCH = 1;

// value type
static const u32 VALUE_TYPE_FOUR_BYTES = 0;
static const u32 VALUE_TYPE_TWO_BYTES = 1;
static const u32 VALUE_TYPE_ONE_BYTE = 2;

// scan type
static const u32 SCAN_TYPE_EQUAL_TO = 0;
static const u32 SCAN_TYPE_NOT_EQUAL_TO = 1;
static const u32 SCAN_TYPE_BIGGER_THAN = 2;
static const u32 SCAN_TYPE_BIGGER_OR_EQUAL = 3;
static const u32 SCAN_TYPE_SMALLER_THAN = 4;
static const u32 SCAN_TYPE_SMALLER_OR_EQUAL = 5;

static bool EqualTo(u32 lhs, u32 rhs) {
    return lhs == rhs;
}

static bool NotEqualTo(u32 lhs, u32 rhs) {
    return lhs != rhs;
}

static bool BiggerThan(u32 lhs, u32 rhs) {
    return lhs > rhs;
}

static bool BiggerOrEqual(u32 lhs, u32 rhs) {
    return lhs >= rhs;
}

static bool SmallerThan(u32 lhs, u32 rhs) {
    return lhs < rhs;
}

static bool SmallerOrEqual(u32 lhs, u32 rhs) {
    return lhs <= rhs;
}

using ScanHandler = bool(*)(u32, u32);
static ScanHandler scan_handlers[6] = {
        &EqualTo, &NotEqualTo, &BiggerThan, &BiggerOrEqual, &SmallerThan, &SmallerOrEqual};

struct SearchSession {
    bool initialized = false;
    u32 start_addr = 0;
    u32 stop_addr = 0;
    u32 value_type = 0;
    u32 search_type = 0;
    u32 scan_type = 0;
    u32 value = 0;
    std::vector<u32> results;
    std::array<u8*, Memory::PAGE_TABLE_NUM_ENTRIES> pointers{};

    void reset() {
        initialized = false;
        start_addr = 0;
        stop_addr = 0;
        value_type = 0;
        search_type = 0;
        scan_type = 0;
        value = 0;
        results.clear();
        for (u32 i = 0; i < pointers.size(); ++i) {
            if (pointers[i] != nullptr) {
                delete[] pointers[i];
                pointers[i] = nullptr;
            }
        }
    }
};

static SearchSession g_search_session{};

template<typename T>
void searchInResults(const SearchSession& session, std::vector<u32>& results) {
    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    for (u32 i = 0; i < session.results.size(); i += 2) {
        u32 addr = session.results[i];
        if (addr < session.start_addr || addr > session.stop_addr) {
            continue;
        }
        u32 old_value = session.results[i + 1];
        u32 page_index = addr >> Memory::PAGE_BITS;
        u32 page_offset = addr & Memory::PAGE_MASK;
        auto p = pagetable->pointers[page_index];
        if (p != nullptr) {
            u32 new_value = *reinterpret_cast<const T*>(p + page_offset);
            if (scan_handlers[session.scan_type](new_value, session.value)) {
                results.push_back(addr);
                results.push_back(new_value);
            }
        }
    }
}

template<typename T>
void searchMemory(SearchSession& session) {
    u32 start = static_cast<u32>(session.start_addr);
    u32 startPage = start >> Memory::PAGE_BITS;
    u32 startOffset = start & Memory::PAGE_MASK;

    u32 stop = static_cast<u32>(session.stop_addr);
    u32 stopPage = stop >> Memory::PAGE_BITS;
    u32 stopOffset = stop & Memory::PAGE_MASK;

    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    for (u32 i = startPage; i <= stopPage; ++i) {
        auto base_ptr = pagetable->pointers[i];
        if (base_ptr != nullptr) {
            const u8* startp = base_ptr + startOffset;
            const u8* stopp = (stopPage == i) ? base_ptr + stopOffset : base_ptr + Memory::PAGE_SIZE;
            const T* p = reinterpret_cast<const T*>(startp);
            const T* stoppT = reinterpret_cast<const T*>(stopp);
            while (p < stoppT) {
                if (scan_handlers[session.scan_type](*p, session.value)) {
                    u32 addr = (i << Memory::PAGE_BITS) + (reinterpret_cast<const u8*>(p) - startp);
                    u32 value32 = *p;
                    session.results.push_back(addr);
                    session.results.push_back(value32);
                }
                p += 1;
            }
        }
        startOffset = 0;
    }
}

template<typename T>
void compareInResults(const SearchSession& session, std::vector<u32>& results) {
    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    for (u32 i = 0; i < session.results.size(); i += 2) {
        u32 addr = session.results[i];
        if (addr < session.start_addr || addr > session.stop_addr) {
            //__android_log_print(ANDROID_LOG_INFO, "zhangwei", "compareInResults continue");
            continue;
        }
        u32 old_value = session.results[i + 1];
        u32 page_index = addr >> Memory::PAGE_BITS;
        u32 page_offset = addr & Memory::PAGE_MASK;
        auto p = pagetable->pointers[page_index];
        if (p != nullptr) {
            u32 new_value = *reinterpret_cast<const T*>(p + page_offset);
            //__android_log_print(ANDROID_LOG_INFO, "zhangwei", "compareInResults new_value: 0x%08X, old_value: 0x%08X", new_value, old_value);
            if (scan_handlers[session.scan_type](new_value, old_value)) {
                results.push_back(addr);
                results.push_back(new_value);
            }
        } else {
            //__android_log_print(ANDROID_LOG_INFO, "zhangwei", "compareInResults p begin to nullptr addr: 0x%08X, old_value: 0x%08X, page_index: %d", addr, old_value, page_index);
        }
    }
}

template<typename T>
void compareMemory(SearchSession& session) {
    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    u32 count = 0;
    for (u32 i = 0; i < g_search_session.results.size(); i += 2) {
        u32 addr = g_search_session.results[i];
        u32 value = g_search_session.results[i + 1];
        if (addr == 0) {
            count ++;
        }
    }
    __android_log_print(ANDROID_LOG_INFO, "zhangwei", "0 addr count ddd: %d, size: %ld", count, g_search_session.results.size());

    for (u32 i = 0 ; i < pagetable->pointers.size(); ++i) {
        auto p = pagetable->pointers[i];
        auto q = session.pointers[i];
        if (p != nullptr) {
            if (q == nullptr) {
                q = new u8[Memory::PAGE_SIZE];
                std::memcpy(q, p, Memory::PAGE_SIZE);
                session.pointers[i] = q;
            } else {
                const T* pT = reinterpret_cast<const T*>(p);
                const T* qT = reinterpret_cast<const T*>(q);
                for (u32 j = 0; j < Memory::PAGE_SIZE; j += sizeof(T)) {
                    if (scan_handlers[session.scan_type](*pT, *qT)) {
                        u32 addr = (i << Memory::PAGE_SIZE) + j;
                        u32 value32 = *pT;

                            __android_log_print(ANDROID_LOG_INFO, "zhangwei", "compareMemory addr: 0x%08X, value32: %d, i: %d, j: %d, size: %ld", addr, value32, i, j, session.results.size());
                        session.results.push_back(addr);
                        session.results.push_back(value32);

                        count = 0;
                        for (u32 k = 0; k < session.results.size(); k += 2) {
                            u32 addr = session.results[k];
                            u32 value = session.results[k + 1];
                            if (addr == 0) {
                                count ++;
                            }
                        }
                        __android_log_print(ANDROID_LOG_INFO, "zhangwei", "0 addr count xxx: %d, size: %ld, i: 0x%04X", count, session.results.size(), i);
                        if (count > 0) {
                            for (u32 k = 0; k < session.results.size(); k += 2) {
                                u32 addr = session.results[k];
                                u32 value = session.results[k + 1];
                                __android_log_print(ANDROID_LOG_INFO, "zhangwei", "addr: %d, value: %d", addr, value);
                            }
                            return;
                        }

                    }
                    pT++;
                    qT++;
                }
            }
        }
    }

    count = 0;
    for (u32 i = 0; i < g_search_session.results.size(); i += 2) {
        u32 addr = g_search_session.results[i];
        u32 value = g_search_session.results[i + 1];
        if (addr == 0) {
            count ++;
        }
    }
    __android_log_print(ANDROID_LOG_INFO, "zhangwei", "0 addr count fff: %d, size: %ld", count, g_search_session.results.size());
}

jintArray searchMemoryRegion(u32 start_addr, u32 stop_addr, u32 value_type, u32 search_type, u32 scan_type, u32 value) {
    if (g_search_session.search_type != search_type ||
        g_search_session.value_type != value_type ||
        g_search_session.start_addr > start_addr ||
        g_search_session.stop_addr < stop_addr) {
        g_search_session.results.clear();
    }

    u32 count = 0;
    for (u32 i = 0; i < g_search_session.results.size(); i += 2) {
        u32 addr = g_search_session.results[i];
        u32 value = g_search_session.results[i + 1];
        if (addr == 0) {
            count ++;
        }
    }
    __android_log_print(ANDROID_LOG_INFO, "zhangwei", "0 addr count aaa: %d", count);

    g_search_session.start_addr = start_addr;
    g_search_session.stop_addr = stop_addr;
    g_search_session.value_type = value_type;
    g_search_session.search_type = search_type;
    g_search_session.scan_type = scan_type;
    g_search_session.value = value;

    if (search_type == SEARCH_TYPE_SPECIFIED_VALUE) {
        if (!g_search_session.initialized) {
            __android_log_print(ANDROID_LOG_INFO, "zhangwei", "searchMemory begin");
            if (g_search_session.value_type == VALUE_TYPE_FOUR_BYTES) {
                searchMemory<u32>(g_search_session);
            } else if (g_search_session.value_type == VALUE_TYPE_TWO_BYTES) {
                searchMemory<u16>(g_search_session);
            } else if (g_search_session.value_type == VALUE_TYPE_ONE_BYTE) {
                searchMemory<u8>(g_search_session);
            }
            g_search_session.initialized = !g_search_session.results.empty();
        } else if (!g_search_session.results.empty()) {
            __android_log_print(ANDROID_LOG_INFO, "zhangwei", "searchInResults begin");
            std::vector<u32> results;
            if (g_search_session.value_type == VALUE_TYPE_FOUR_BYTES) {
                searchInResults<u32>(g_search_session, results);
            } else if (g_search_session.value_type == VALUE_TYPE_TWO_BYTES) {
                searchInResults<u16>(g_search_session, results);
            } else if (g_search_session.value_type == VALUE_TYPE_ONE_BYTE) {
                searchInResults<u8>(g_search_session, results);
            }
            g_search_session.results.swap(results);
        }
    } else if (search_type == SEARCH_TYPE_UNKNOWN_SEARCH) {
        if (!g_search_session.initialized) {
            __android_log_print(ANDROID_LOG_INFO, "zhangwei", "compareMemory begin");
            if (g_search_session.value_type == VALUE_TYPE_FOUR_BYTES) {
                compareMemory<u32>(g_search_session);
            } else if (g_search_session.value_type == VALUE_TYPE_TWO_BYTES) {
                compareMemory<u16>(g_search_session);
            } else if (g_search_session.value_type == VALUE_TYPE_ONE_BYTE) {
                compareMemory<u8>(g_search_session);
            }
            g_search_session.initialized = !g_search_session.results.empty();
        } else if (!g_search_session.results.empty()) {
            __android_log_print(ANDROID_LOG_INFO, "zhangwei", "compareInResults begin");
            std::vector<u32> results;
            if (g_search_session.value_type == VALUE_TYPE_FOUR_BYTES) {
                compareInResults<u32>(g_search_session, results);
            } else if (g_search_session.value_type == VALUE_TYPE_TWO_BYTES) {
                compareInResults<u16>(g_search_session, results);
            } else if (g_search_session.value_type == VALUE_TYPE_ONE_BYTE) {
                compareInResults<u8>(g_search_session, results);
            }
            g_search_session.results.swap(results);
        }
    }

    count = 0;
    for (u32 i = 0; i < g_search_session.results.size(); i += 2) {
        u32 addr = g_search_session.results[i];
        u32 value = g_search_session.results[i + 1];
        if (addr == 0) {
            count ++;
        }
    }
    __android_log_print(ANDROID_LOG_INFO, "zhangwei", "0 addr count bbb: %d", count);

    return ToJIntArray(g_search_session.results.data(), g_search_session.results.size());
}

jintArray getSearchResults() {
    u32 count = 0;
    for (u32 i = 0; i < g_search_session.results.size(); i += 2) {
        u32 addr = g_search_session.results[i];
        u32 value = g_search_session.results[i + 1];
        if (addr == 0) {
            count ++;
        }
    }
    __android_log_print(ANDROID_LOG_INFO, "zhangwei", "0 addr count ccc: %d", count);

    return ToJIntArray(g_search_session.results.data(), g_search_session.results.size());
}

void resetSearchResults() {
    g_search_session.reset();
}
