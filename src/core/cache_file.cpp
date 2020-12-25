
#include <android/log.h>
#include <minilzo.h>

#include "core/cache_file.h"

/* We want to compress the data block at 'in' with length 'IN_LEN' to
 * the block at 'out'. Because the input block may be incompressible,
 * we must provide a little more output space in case that compression
 * is not possible.
 */

#define IN_LEN (128 * 1024ul)
#define OUT_LEN (IN_LEN + IN_LEN / 16 + 64 + 3)

/* Work-memory needed for compression. Allocate memory in units
 * of 'lzo_align_t' (instead of 'char') to make sure it is properly aligned.
 */

#define HEAP_ALLOC(var, size)                                                                      \
    lzo_align_t __LZO_MMODEL var[((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t)]

static bool s_lzo_init = false;

namespace Core {

class CacheFile::CacheFileImpl {
public:
    CacheFileImpl(FileUtil::IOFile& file) : file(file) {
        if (!s_lzo_init) {
            if (lzo_init() != LZO_E_OK) {
                ASSERT_MSG(false, "Internal LZO Error - lzo_init() failed");
            }
            s_lzo_init = true;
        }
        buffer.reserve(OUT_LEN);
    }

    ~CacheFileImpl() {
        if (write_size > 0 && buffer.size() > 0) {
            WriteData(buffer.data(), buffer.size());
        }
    }

    void SaveData(u8* data, u32 size) {
        std::size_t buffer_size = buffer.size();
        write_size += size;

        if (buffer_size + size < OUT_LEN) {
            buffer.insert(buffer.end(), data, data + size);
            return;
        }

        if (buffer_size > 0) {
            u32 saved_size = OUT_LEN - buffer_size;
            u8* data_end = data + saved_size;
            buffer.insert(buffer.end(), data, data_end);
            WriteData(buffer.data(), OUT_LEN);
            buffer.clear();
            data = data_end;
            size -= saved_size;
        }

        while (size > OUT_LEN) {
            WriteData(data, OUT_LEN);
            size -= OUT_LEN;
            data += OUT_LEN;
        }

        if (size > 0) {
            buffer.insert(buffer.end(), data, data + size);
        }
    }

    void WriteData(const u8* data, u32 size) {
        lzo_uint out_len = 0;
        if (lzo1x_1_compress(data, size, out, &out_len, wrkmem) != LZO_E_OK) {
            ASSERT_MSG(false, "Internal LZO Error - compression failed");
        }

        file.WriteArray((lzo_uint32*)&out_len, 1);
        file.WriteBytes(out, out_len);
    }

    void LoadData(u8* data, u32 size) {
        std::size_t buffer_size = buffer.size() - read_cursor;
        if (buffer_size >= size) {
            std::memcpy(data, buffer.data() + read_cursor, size);
            read_cursor += size;
            return;
        }

        if (buffer_size > 0) {
            std::memcpy(data, buffer.data() + read_cursor, buffer_size);
            size -= buffer_size;
            data += buffer_size;
        }

        if (size > 0 && ReadData()) {
            LoadData(data, size);
        }
    }

    bool ReadData() {
        lzo_uint32 cur_len = 0;
        lzo_uint new_len = 0;
        if (!file.ReadArray(&cur_len, 1)) {
            ASSERT_MSG(false, "read length failed!");
            return false;
        }

        read_cursor = 0;
        buffer.resize(OUT_LEN);
        file.ReadBytes(out, cur_len);
        const int res = lzo1x_decompress(out, cur_len, buffer.data(), &new_len, wrkmem);
        if (res != LZO_E_OK) {
            ASSERT_MSG(false, "Internal LZO Error - decompression failed ({}) ({}, {})", res, i,
                       new_len);
            buffer.clear();
            return false;
        }

        buffer.resize(new_len);
        return new_len > 0;
    }

private:
    std::size_t write_size = 0;
    std::size_t read_cursor = 0;
    std::vector<u8> buffer;
    FileUtil::IOFile& file;
    // minilzo
    unsigned char __LZO_MMODEL out[OUT_LEN];
    HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);
};

CacheFile::CacheFile(const std::string& filename, Mode mode)
    : file(filename, mode == MODE_LOAD ? "rb" : "wb"), mode(mode) {
    impl = std::make_shared<CacheFileImpl>(file);
}

CacheFile::~CacheFile() {
    impl.reset();
}

void CacheFile::DoVoid(void* data, u32 size) {
    switch (mode) {
    case MODE_LOAD:
        impl->LoadData(reinterpret_cast<u8*>(data), size);
        break;

    case MODE_SAVE:
        impl->SaveData(reinterpret_cast<u8*>(data), size);
        break;

    case MODE_SKIP:
        break;
    }

    if (!file.IsGood()) {
        mode = MODE_SKIP;
    }
}

} // namespace Core
