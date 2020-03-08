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
    u32 start_addr = 0;
    u32 stop_addr = 0;
    u32 value_type = 0;
    u32 search_type = 0;
    u32 scan_type = 0;
    u32 value = 0;
    std::vector<u32> results;
};

static SearchSession g_search_session{};

void searchInResults(const SearchSession& session, std::vector<u32>& results) {
    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    if (session.value_type == VALUE_TYPE_FOUR_BYTES) {
        for (u32 i = 0; i < session.results.size(); i += 2) {
            u32 addr = session.results[i];
            if (addr < session.start_addr || addr > session.stop_addr) {
                continue;
            }
            u32 old_value = session.results[i + 1];
            u32 page_index = addr >> Memory::PAGE_BITS;
            u32 page_offset = addr & Memory::PAGE_MASK;
            auto base_ptr = pagetable->pointers[page_index];
            if (base_ptr != nullptr) {
                u32 new_value = *reinterpret_cast<const u32*>(base_ptr + page_offset);
                if (scan_handlers[session.scan_type](new_value, session.value)) {
                    results.push_back(addr);
                    results.push_back(new_value);
                }
            }
        }
    } else if (session.value_type == VALUE_TYPE_TWO_BYTES) {
        for (u32 i = 0; i < session.results.size(); i += 2) {
            u32 addr = session.results[i];
            if (addr < session.start_addr || addr > session.stop_addr) {
                continue;
            }
            u32 old_value = session.results[i + 1];
            u32 page_index = addr >> Memory::PAGE_BITS;
            u32 page_offset = addr & Memory::PAGE_MASK;
            auto base_ptr = pagetable->pointers[page_index];
            if (base_ptr != nullptr) {
                u32 new_value = *reinterpret_cast<const u16*>(base_ptr + page_offset);
                if (scan_handlers[session.scan_type](new_value, session.value)) {
                    results.push_back(addr);
                    results.push_back(new_value);
                }
            }
        }
    } else if (session.value_type == VALUE_TYPE_ONE_BYTE) {
        for (u32 i = 0; i < session.results.size(); i += 2) {
            u32 addr = session.results[i];
            if (addr < session.start_addr || addr > session.stop_addr) {
                continue;
            }
            u32 old_value = session.results[i + 1];
            u32 page_index = addr >> Memory::PAGE_BITS;
            u32 page_offset = addr & Memory::PAGE_MASK;
            auto base_ptr = pagetable->pointers[page_index];
            if (base_ptr != nullptr) {
                u32 new_value = *reinterpret_cast<const u8*>(base_ptr + page_offset);
                if (scan_handlers[session.scan_type](new_value, session.value)) {
                    results.push_back(addr);
                    results.push_back(new_value);
                }
            }
        }
    }
}

void searchInMemory(const SearchSession& session, std::vector<u32>& results) {
    u32 start = static_cast<u32>(session.start_addr);
    u32 startPage = start >> Memory::PAGE_BITS;
    u32 startOffset = start & Memory::PAGE_MASK;

    u32 stop = static_cast<u32>(session.stop_addr);
    u32 stopPage = stop >> Memory::PAGE_BITS;
    u32 stopOffset = stop & Memory::PAGE_MASK;

    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    if (session.value_type == VALUE_TYPE_FOUR_BYTES) {
        for (u32 i = startPage; i <= stopPage; ++i) {
            auto base_ptr = pagetable->pointers[i];
            if (base_ptr != nullptr) {
                const u8* startp = base_ptr + startOffset;
                const u8* stopp = (stopPage == i) ? base_ptr + stopOffset : base_ptr + Memory::PAGE_SIZE;
                const u32* p = reinterpret_cast<const u32*>(startp);
                const u32* stopp32 = reinterpret_cast<const u32*>(stopp);
                while (p < stopp32) {
                    if (scan_handlers[session.scan_type](*p, session.value)) {
                        u32 addr = (i << Memory::PAGE_BITS) + (reinterpret_cast<const u8*>(p) - startp);
                        u32 value32 = *p;
                        results.push_back(addr);
                        results.push_back(value32);
                    }
                    p += 1;
                }
            }
            startOffset = 0;
        }
    } else if (session.value_type == VALUE_TYPE_TWO_BYTES) {
        for (u32 i = startPage; i <= stopPage; ++i) {
            auto base_ptr = pagetable->pointers[i];
            if (base_ptr != nullptr) {
                const u8* startp = base_ptr + startOffset;
                const u8* stopp = (stopPage == i) ? base_ptr + stopOffset : base_ptr + Memory::PAGE_SIZE;
                const u16* p = reinterpret_cast<const u16*>(startp);
                const u16* stopp16 = reinterpret_cast<const u16*>(stopp);
                while (p < stopp16) {
                    if (scan_handlers[session.scan_type](*p, session.value)) {
                        u32 addr = (i << Memory::PAGE_BITS) + (reinterpret_cast<const u8*>(p) - startp);
                        u32 value32 = *p;
                        results.push_back(addr);
                        results.push_back(value32);
                    }
                    p += 1;
                }
            }
            startOffset = 0;
        }
    } else if (session.value_type == VALUE_TYPE_ONE_BYTE) {
        for (u32 i = startPage; i <= stopPage; ++i) {
            auto base_ptr = pagetable->pointers[i];
            if (base_ptr != nullptr) {
                const u8* startp = base_ptr + startOffset;
                const u8* stopp = (stopPage == i) ? base_ptr + stopOffset : base_ptr + Memory::PAGE_SIZE;
                const u8* p = startp;
                while (p < stopp) {
                    if (scan_handlers[session.scan_type](*p, session.value)) {
                        u32 addr = (i << Memory::PAGE_BITS) + (reinterpret_cast<const u8*>(p) - startp);
                        u32 value32 = *p;
                        results.push_back(addr);
                        results.push_back(value32);
                    }
                    p += 1;
                }
            }
            startOffset = 0;
        }
    }
}

jintArray searchMemoryRegion(u32 start_addr, u32 stop_addr, u32 value_type, u32 search_type, u32 scan_type, u32 value) {
    if (g_search_session.value_type != value_type ||
        g_search_session.start_addr > start_addr ||
        g_search_session.stop_addr < stop_addr) {
        g_search_session.results.clear();
    }

    g_search_session.start_addr = start_addr;
    g_search_session.stop_addr = stop_addr;
    g_search_session.value_type = value_type;
    g_search_session.search_type = search_type;
    g_search_session.scan_type = scan_type;
    g_search_session.value = value;

    if (search_type == SEARCH_TYPE_SPECIFIED_VALUE) {
        if (g_search_session.results.empty()) {
            searchInMemory(g_search_session, g_search_session.results);
        } else {
            std::vector<u32> results;
            searchInResults(g_search_session, results);
            g_search_session.results.swap(results);
        }
    } else if (search_type == SEARCH_TYPE_UNKNOWN_SEARCH) {
        // todo
    }

    return ToJIntArray(g_search_session.results.data(), g_search_session.results.size());
}

jintArray getSearchResults() {
    return ToJIntArray(g_search_session.results.data(), g_search_session.results.size());
}

void resetSearchResults() {
    g_search_session.start_addr = 0;
    g_search_session.stop_addr = 0;
    g_search_session.value_type = 0;
    g_search_session.search_type = 0;
    g_search_session.scan_type = 0;
    g_search_session.value = 0;
    g_search_session.results.clear();
}
