// Copyright 2013 Dolphin Emulator Project / 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <cstdio>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include "common/common_types.h"

namespace FileUtil {

// User paths for GetUserPath
enum class UserPath {
    AmiiboDir,
    CacheDir,
    CheatsDir,
    ConfigDir,
    DLLDir,
    DumpDir,
    LoadDir,
    LogDir,
    NANDDir,
    RootDir,
    SDMCDir,
    ShaderDir,
    StatesDir,
    SysDataDir,
    UserDir,
};

// FileSystem tree node/
struct FSTEntry {
    bool isDirectory;
    u64 size;                 // file length or number of entries from children
    std::string physicalName; // name on disk
    std::string virtualName;  // name in FST names table
    std::vector<FSTEntry> children;
};

// Returns true if file filename exists
bool Exists(const std::string& filename);

// Returns true if filename is a directory
bool IsDirectory(const std::string& filename);

// Returns the size of filename (64bit)
u64 GetSize(const std::string& filename);

// get file modification timestamp, return 0 on failed
u64 GetFileModificationTimestamp(const std::string& filename);

// Returns true if successful, or path already exists.
bool CreateDir(const std::string& filename);

// Creates the full path of fullPath returns true on success
bool CreateFullPath(const std::string& fullPath);

// Deletes a given filename, return true on success
// Doesn't supports deleting a directory
bool Delete(const std::string& filename);

// Deletes a directory filename, returns true on success
bool DeleteDir(const std::string& filename);

// renames file srcFilename to destFilename, returns true on success
bool Rename(const std::string& srcFilename, const std::string& destFilename);

// copies file srcFilename to destFilename, returns true on success
bool Copy(const std::string& srcFilename, const std::string& destFilename);

// creates an empty file filename, returns true on success
bool CreateEmptyFile(const std::string& filename);

/**
 * @param num_entries_out to be assigned by the callable with the number of iterated directory
 * entries, never null
 * @param directory the path to the enclosing directory
 * @param virtual_name the entry name, without any preceding directory info
 * @return whether handling the entry succeeded
 */
using DirectoryEntryCallable = std::function<bool(
    u64* num_entries_out, const std::string& directory, const std::string& virtual_name)>;

/**
 * Scans a directory, calling the callback for each file/directory contained within.
 * If the callback returns failure, scanning halts and this function returns failure as well
 * @param num_entries_out assigned by the function with the number of iterated directory entries,
 * can be null
 * @param directory the directory to scan
 * @param callback The callback which will be called for each entry
 * @return whether scanning the directory succeeded
 */
bool ForeachDirectoryEntry(u64* num_entries_out, const std::string& directory,
                           const DirectoryEntryCallable& callback);

/**
 * Scans the directory tree, storing the results.
 * @param directory the parent directory to start scanning from
 * @param parent_entry FSTEntry where the filesystem tree results will be stored.
 * @param recursion Number of children directories to read before giving up.
 * @return the total number of files/directories found
 */
u64 ScanDirectoryTree(const std::string& directory, FSTEntry& parent_entry,
                      unsigned int recursion = 0);

/**
 * Recursively searches through a FSTEntry for files, and stores them.
 * @param directory The FSTEntry to start scanning from
 * @param parent_entry FSTEntry vector where the results will be stored.
 */
void GetAllFilesFromNestedEntries(FSTEntry& directory, std::vector<FSTEntry>& output);

// deletes the given directory and anything under it. Returns true on success.
bool DeleteDirRecursively(const std::string& directory, unsigned int recursion = 256);

// Returns the current directory
std::optional<std::string> GetCurrentDir();

// Create directory and copy contents (does not overwrite existing files)
void CopyDir(const std::string& source_path, const std::string& dest_path);

void SetUserPath(const std::string& path);

// Returns a pointer to a string with a Citra data dir in the user's home
// directory. To be used in "multi-user" mode (that is, installed).
const std::string& GetUserPath(UserPath path);

std::size_t WriteStringToFile(bool text_file, const std::string& filename, std::string_view str);

std::size_t ReadFileToString(bool text_file, const std::string& filename, std::string& str);

/**
 * Splits the filename into 8.3 format
 * Loosely implemented following https://en.wikipedia.org/wiki/8.3_filename
 * @param filename The normal filename to use
 * @param short_name A 9-char array in which the short name will be written
 * @param extension A 4-char array in which the extension will be written
 */
void SplitFilename83(const std::string& filename, std::array<char, 9>& short_name,
                     std::array<char, 4>& extension);

// Splits the path on '/' or '\' and put the components into a vector
// i.e. "C:\Users\Yuzu\Documents\save.bin" becomes {"C:", "Users", "Yuzu", "Documents", "save.bin" }
std::vector<std::string> SplitPathComponents(std::string_view filename);

// Gets all of the text up to the last '/' or '\' in the path.
std::string_view GetParentPath(std::string_view path);

// Gets the filename of the path
std::string_view GetFilename(std::string_view path);

class IOHandler {
public:
    virtual ~IOHandler() = default;
    virtual std::size_t Read(void* buf, std::size_t size, std::size_t count) = 0;
    virtual std::size_t Write(const void* buf, std::size_t size, std::size_t count) = 0;
    virtual bool Seek(s64 offset, int whence) = 0;
    virtual u64 Tell() = 0;
    virtual u64 GetSize() = 0;
    virtual bool Resize(u64 size) = 0;
    virtual bool Flush() = 0;
};

class IOFactory {
public:
    virtual ~IOFactory() = default;
    virtual std::unique_ptr<IOHandler> Open(const std::string& filename, const char openmode[]) = 0;
};
void RegisterIOFactory(std::unique_ptr<IOFactory> factory);

// simple wrapper for cstdlib file functions to
// hopefully will make error checking easier
// and make forgetting an fclose() harder
class IOFile : public NonCopyable {
public:
    IOFile() = default;
    IOFile(const std::string& filename, const char openmode[]);
    IOFile(IOFile&& other) noexcept;
    ~IOFile();

    bool Open(const std::string& filename, const char openmode[]);

    void Close() {
        m_file.reset();
        m_good = false;
    }

    void ReadAllLines(std::vector<std::string>& lines);

    template <typename T>
    std::size_t ReadArray(T* data, std::size_t length) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "Given array does not consist of trivially copyable objects");
        if (!IsOpen()) {
            return std::numeric_limits<std::size_t>::max();
        }
        std::size_t items_read = m_file->Read(data, sizeof(T), length);
        if (items_read != length)
            m_good = false;
        return items_read;
    }

    template <typename T>
    std::size_t WriteArray(const T* data, std::size_t length) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "Given array does not consist of trivially copyable objects");
        if (!IsOpen()) {
            return std::numeric_limits<std::size_t>::max();
        }
        std::size_t items_written = m_file->Write(data, sizeof(T), length);
        if (items_written != length)
            m_good = false;
        return items_written;
    }

    template <typename T>
    std::size_t ReadBytes(T* data, std::size_t length) {
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        return ReadArray(reinterpret_cast<char*>(data), length);
    }

    template <typename T>
    std::size_t WriteBytes(const T* data, std::size_t length) {
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        return WriteArray(reinterpret_cast<const char*>(data), length);
    }

    template <typename T>
    std::size_t WriteObject(const T& object) {
        static_assert(!std::is_pointer_v<T>, "WriteObject arguments must not be a pointer");
        return WriteArray(&object, 1);
    }

    std::size_t WriteString(std::string_view str) {
        return WriteArray(str.data(), str.length());
    }

    bool IsOpen() const {
        return nullptr != m_file;
    }

    // m_good is set to false when a read, write or other function fails
    bool IsGood() const {
        return m_good;
    }
    explicit operator bool() const {
        return IsGood();
    }

    bool Seek(s64 offset, int whence) {
        return m_file->Seek(offset, whence);
    }

    u64 Tell() const {
        m_file->Tell();
    }

    u64 GetSize() const {
        return m_file->GetSize();
    }

    bool Resize(u64 size) {
        return m_file->Resize(size);
    }

    bool Flush() {
        return m_file->Flush();
    }

private:
    std::unique_ptr<FileUtil::IOHandler> m_file;
    bool m_good = false;
};

} // namespace FileUtil

// To deal with Windows being dumb at unicode:
template <typename T>
void OpenFStream(T& fstream, const std::string& filename, std::ios_base::openmode openmode) {
#ifdef _MSC_VER
    fstream.open(Common::UTF8ToUTF16W(filename), openmode);
#else
    fstream.open(filename, openmode);
#endif
}
