#include <unistd.h>
#include <sys/stat.h>

#include "saf_handler.h"
#include "jni_common.h"

class SAFHandler : public FileUtil::IOHandler {
public:
    static std::unique_ptr<FileUtil::IOHandler> Open(const std::string& filename, const char openmode[]) {
        int fd = NativeLibrary::SafOpen(filename, openmode);
        if (fd != 0) {
            return std::make_unique<SAFHandler>(fd);
        }
        return nullptr;
    }

    explicit SAFHandler(int fd) : m_fd(fd) {

    }

    ~SAFHandler() override {
        NativeLibrary::SafClose(m_fd);
    }

    std::size_t Read(void* buf, std::size_t size, std::size_t count) override {
        return read(m_fd, buf, size * count);
    }

    std::size_t Write(const void* buf, std::size_t size, std::size_t count) override {
        return write(m_fd, buf, size * count);
    }

    bool Seek(s64 offset, int whence) override {
        return -1 != lseek(m_fd, offset, whence);
    }

    u64 Tell() override {
        return lseek(m_fd, 0, SEEK_CUR);
    }

    u64 GetSize() override {
        struct stat buf;
        if (fstat(m_fd, &buf) == 0) {
            return buf.st_size;
        }
        return 0;
    }

    bool Resize(u64 size) override {
        return false;
    }

    bool Flush() override {
        return 0 == fsync(m_fd);
    }
private:
    int m_fd;
};

class STDHandler : public FileUtil::IOHandler {
public:
    static std::unique_ptr<FileUtil::IOHandler> Open(const std::string& filename, const char openmode[]) {
        std::FILE* file = std::fopen(filename.c_str(), openmode);
        if (file != nullptr) {
            return std::make_unique<STDHandler>(file);
        }
        return nullptr;
    }

    explicit STDHandler(std::FILE* file) : m_file(file) {

    }

    ~STDHandler() override {
        std::fclose(m_file);
    }

    std::size_t Read(void* buf, std::size_t size, std::size_t count) override {
        return std::fread(buf, size, count, m_file);
    }

    std::size_t Write(const void* buf, std::size_t size, std::size_t count) override {
        return std::fwrite(buf, size, count, m_file);
    }

    bool Seek(s64 offset, int whence) override {
        return 0 != fseeko(m_file, offset, whence);
    }

    u64 Tell() override {
        return ftello(m_file);
    }

    u64 GetSize() override {
        /*u64 size = 0;
        auto offset = Tell();
        if (Seek(0, SEEK_END)) {
            size = Tell();
            Seek(offset, SEEK_SET);
        }
        return size;*/
        struct stat buf;
        if (fstat(fileno(m_file), &buf) == 0) {
            return buf.st_size;
        }
        return 0;
    }

    bool Resize(u64 size) override {
        return 0 == ftruncate(fileno(m_file), size);
    }

    bool Flush() override {
        return 0 == std::fflush(m_file);
    }
private:
    std::FILE* m_file;
};

std::unique_ptr<FileUtil::IOHandler> AndroidIOFactory::Open(const std::string& filename, const char openmode[]) {
    if (IsSafPath(filename)) {
        return SAFHandler::Open(filename, openmode);
    } else {
        return STDHandler::Open(filename, openmode);
    }
}

bool IsSafPath(const std::string& path) {
    std::string_view prefix{"content://"};
    bool is_saf = true;
    if (path.size() > prefix.size()) {
        for (int i = 0; i < prefix.size(); ++i) {
            if (path[i] != prefix[i]) {
                is_saf = false;
                break;
            }
        }
    } else {
        is_saf = false;
    }
    return is_saf;
}
