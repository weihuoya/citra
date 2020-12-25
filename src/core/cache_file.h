#pragma once

#include <array>
#include <cstddef>
#include <cstring>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/assert.h"
#include "common/common_types.h"
#include "common/file_util.h"
#include "common/logging/log.h"

namespace Core {

// XXX: Replace this with std::is_trivially_copyable<T> once we stop using volatile
// on things that are put in savestates, as volatile types are not trivially copyable.
template <typename T>
constexpr bool IsTriviallyCopyable = std::is_trivially_copyable<std::remove_volatile_t<T>>::value;

class CacheFile {
public:
    enum Mode {
        MODE_LOAD = 1,
        MODE_SAVE,
        MODE_SKIP,
    };

    CacheFile(const std::string& filename, Mode mode);
    ~CacheFile();

    template <typename K, class V>
    void Do(std::map<K, V>& x) {
        u32 count = static_cast<u32>(x.size());
        Do(count);

        switch (mode) {
        case MODE_LOAD:
            for (x.clear(); count != 0; --count) {
                std::pair<K, V> pair;
                Do(pair.first);
                Do(pair.second);
                x.insert(pair);
            }
            break;

        case MODE_SAVE:
            for (auto& elem : x) {
                Do(elem.first);
                Do(elem.second);
            }
            break;
        }
    }

    template <typename K, class V>
    void Do(std::unordered_map<K, V>& x) {
        u32 count = static_cast<u32>(x.size());
        Do(count);

        switch (mode) {
        case MODE_LOAD:
            for (x.clear(); count != 0; --count) {
                K key;
                V val;
                Do(key);
                Do(val);
                x[key] = std::move(val);
            }
            break;

        case MODE_SAVE:
            for (auto& elem : x) {
                Do(elem.first);
                Do(elem.second);
            }
            break;
        }
    }

    template <typename V>
    void Do(std::set<V>& x) {
        u32 count = static_cast<u32>(x.size());
        Do(count);

        switch (mode) {
        case MODE_LOAD:
            for (x.clear(); count != 0; --count) {
                V value;
                Do(value);
                x.insert(value);
            }
            break;

        case MODE_SAVE:
            for (const V& val : x) {
                Do(val);
            }
            break;
        }
    }

    template <typename V>
    void Do(std::unordered_set<V>& x) {
        u32 count = x.size();
        Do(count);

        switch (mode) {
        case MODE_LOAD:
            for (x.clear(); count != 0; --count) {
                V value;
                Do(value);
                x.insert(value);
            }
            break;

        case MODE_SAVE:
            for (const V& val : x) {
                Do(val);
            }
            break;
        }
    }

    template <typename T>
    void Do(std::vector<T>& x) {
        DoContiguousContainer(x);
    }

    template <typename T>
    void Do(std::list<T>& x) {
        DoContainer(x);
    }

    template <typename T>
    void Do(std::deque<T>& x) {
        DoContainer(x);
    }

    template <typename T>
    void Do(std::basic_string<T>& x) {
        DoContiguousContainer(x);
    }

    template <typename T, typename U>
    void Do(std::pair<T, U>& x) {
        Do(x.first);
        Do(x.second);
    }

    template <typename T, std::size_t N>
    void DoArray(std::array<T, N>& x) {
        DoArray(x.data(), static_cast<u32>(x.size()));
    }

    template <typename T, typename std::enable_if_t<IsTriviallyCopyable<T>, int> = 0>
    void DoArray(T* x, u32 count) {
        DoVoid(x, count * sizeof(T));
    }

    template <typename T, typename std::enable_if_t<!IsTriviallyCopyable<T>, int> = 0>
    void DoArray(T* x, u32 count) {
        for (u32 i = 0; i < count; ++i)
            Do(x[i]);
    }

    template <typename T, std::size_t N>
    void DoArray(T (&arr)[N]) {
        DoArray(arr, static_cast<u32>(N));
    }

    template <typename T>
    void Do(std::atomic<T>& atomic) {
        T temp = atomic.load();
        Do(temp);
        if (mode == MODE_LOAD)
            atomic.store(temp);
    }

    template <typename T>
    void Do(T& x) {
        static_assert(IsTriviallyCopyable<T>, "Only sane for trivially copyable types");
        // Note:
        // Usually we can just use x = **ptr, etc.  However, this doesn't work
        // for unions containing BitFields (long story, stupid language rules)
        // or arrays.  This will get optimized anyway.
        DoVoid((void*)&x, sizeof(x));
    }

    template <typename T>
    void DoPOD(T& x) {
        DoVoid((void*)&x, sizeof(x));
    }

    void Do(bool& x) {
        // bool's size can vary depending on platform, which can
        // cause breakages. This treats all bools as if they were
        // 8 bits in size.
        u8 stable = static_cast<u8>(x);

        Do(stable);

        if (mode == MODE_LOAD)
            x = stable != 0;
    }

    template <typename T>
    void DoPointer(T*& x, T* const base) {
        // pointers can be more than 2^31 apart, but you're using this function wrong if you need
        // that much range
        ptrdiff_t offset = x - base;
        Do(offset);
        if (mode == MODE_LOAD) {
            x = base + offset;
        }
    }

    void DoMarker(const std::string& prevName, u32 arbitraryNumber = 0x42) {
        u32 cookie = arbitraryNumber;
        Do(cookie);

        if (mode == MODE_LOAD && cookie != arbitraryNumber) {
            LOG_CRITICAL(
                Core, "Error: After \"{}\", found {} instead of save marker {}. Aborting load...",
                prevName.c_str(), cookie, arbitraryNumber);
            mode = MODE_SKIP;
        }
    }

    template <typename T>
    void DoHeader(T& data) {
        switch (mode) {
        case MODE_LOAD:
            file.ReadBytes(&data, sizeof(data));
            break;
        case MODE_SAVE:
            file.WriteBytes(&data, sizeof(data));
            break;
        case MODE_SKIP:
            break;
        }

        if (!file.IsGood()) {
            mode = MODE_SKIP;
        }
    }

    template <typename T, typename Functor>
    void DoEachElement(T& container, Functor member) {
        u32 size = static_cast<u32>(container.size());
        Do(size);
        container.resize(size);

        for (auto& elem : container)
            member(*this, elem);
    }

    bool IsGood() const {
        return mode != MODE_SKIP && file.IsGood();
    }

private:
    template <typename T>
    void DoContiguousContainer(T& container) {
        u32 size = static_cast<u32>(container.size());
        Do(size);
        container.resize(size);

        if (size > 0)
            DoArray(&container[0], size);
    }

    template <typename T>
    void DoContainer(T& x) {
        DoEachElement(x, [](CacheFile& p, typename T::value_type& elem) { p.Do(elem); });
    }

    void DoVoid(void* data, u32 size);

private:
    class CacheFileImpl;
    std::shared_ptr<CacheFileImpl> impl;
    FileUtil::IOFile file;
    Mode mode;
};

} // namespace Core
