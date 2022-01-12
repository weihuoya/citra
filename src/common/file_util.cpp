// Copyright 2013 Dolphin Emulator Project / 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <limits>
#include <memory>
#include <sstream>
#include <unordered_map>
#include "common/assert.h"
#include "common/common_funcs.h"
#include "common/common_paths.h"
#include "common/file_util.h"
#include "common/logging/log.h"

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <pwd.h>
#include <unistd.h>

#include <algorithm>
#include <utility>
#include <sys/stat.h>

// This namespace has various generic functions related to files and paths.
// The code still needs a ton of cleanup.
// REMEMBER: strdup considered harmful!
namespace FileUtil {

static std::unique_ptr<IOFactory> s_io_factory;
void RegisterIOFactory(std::unique_ptr<IOFactory> factory) {
    s_io_factory = std::move(factory);
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

// Remove any ending forward slashes from directory paths
// Modifies argument.
static int GetFileStat(const std::string& filename, struct stat* buf) {
    std::size_t i = filename.length();
    if (i > 1 && filename[i - 1] == DIR_SEP_CHR) {
        while (i > 0 && filename[i - 1] == DIR_SEP_CHR) {
            --i;
        }
        return stat(filename.substr(0, i).c_str(), buf);
    } else {
        return stat(filename.c_str(), buf);
    }
}

bool Exists(const std::string& filename) {
    struct stat file_info;
    return (GetFileStat(filename, &file_info) == 0);
}

bool IsDirectory(const std::string& filename) {
    struct stat file_info;
    if (GetFileStat(filename.c_str(), &file_info) != 0) {
        return false;
    }
    return S_ISDIR(file_info.st_mode);
}

u64 GetSize(const std::string& filename) {
    struct stat file_info;
    if (stat(filename.c_str(), &file_info) != 0) {
        LOG_ERROR(Common_Filesystem, "failed {}: No such file", filename);
        return false;
    }
    return file_info.st_size;
}

u64 GetFileModificationTimestamp(const std::string& filename) {
    struct stat64 file_info;
    if (stat64(filename.c_str(), &file_info) != 0) {
        LOG_ERROR(Common_Filesystem, "failed {}: No such file", filename);
        return 0;
    }
    return static_cast<u64>(file_info.st_mtim.tv_sec);
}

bool Delete(const std::string& filename) {
    LOG_TRACE(Common_Filesystem, "file {}", filename);

    // Return true because we care about the file no
    // being there, not the actual delete.
    if (!Exists(filename)) {
        LOG_DEBUG(Common_Filesystem, "{} does not exist", filename);
        return true;
    }

    // We can't delete a directory
    if (IsDirectory(filename)) {
        LOG_ERROR(Common_Filesystem, "Failed: {} is a directory", filename);
        return false;
    }

    if (unlink(filename.c_str()) == -1) {
        LOG_ERROR(Common_Filesystem, "unlink failed on {}: {}", filename, GetLastErrorMsg());
        return false;
    }

    return true;
}

bool CreateDir(const std::string& path) {
    LOG_TRACE(Common_Filesystem, "directory {}", path);
    if (mkdir(path.c_str(), 0755) == 0)
        return true;

    int err = errno;

    if (err == EEXIST) {
        LOG_DEBUG(Common_Filesystem, "mkdir failed on {}: already exists", path);
        return true;
    }

    LOG_ERROR(Common_Filesystem, "mkdir failed on {}: {}", path, strerror(err));
    return false;
}

bool CreateFullPath(const std::string& fullPath) {
    int panicCounter = 100;
    LOG_TRACE(Common_Filesystem, "path {}", fullPath);

    if (FileUtil::Exists(fullPath)) {
        LOG_DEBUG(Common_Filesystem, "path exists {}", fullPath);
        return true;
    }

    std::size_t position = 0;
    while (true) {
        // Find next sub path
        position = fullPath.find(DIR_SEP_CHR, position);

        // we're done, yay!
        if (position == fullPath.npos)
            return true;

        // Include the '/' so the first call is CreateDir("/") rather than CreateDir("")
        std::string const subPath(fullPath.substr(0, position + 1));
        if (!FileUtil::IsDirectory(subPath) && !FileUtil::CreateDir(subPath)) {
            LOG_ERROR(Common, "CreateFullPath: directory creation failed");
            return false;
        }

        // A safety check
        panicCounter--;
        if (panicCounter <= 0) {
            LOG_ERROR(Common, "CreateFullPath: directory structure is too deep");
            return false;
        }
        position++;
    }
}

bool DeleteDir(const std::string& filename) {
    LOG_TRACE(Common_Filesystem, "directory {}", filename);

    // check if a directory
    if (!FileUtil::IsDirectory(filename)) {
        LOG_ERROR(Common_Filesystem, "Not a directory {}", filename);
        return false;
    }

    if (rmdir(filename.c_str()) == 0)
        return true;

    LOG_ERROR(Common_Filesystem, "failed {}: {}", filename, GetLastErrorMsg());

    return false;
}

bool Rename(const std::string& srcFilename, const std::string& destFilename) {
    LOG_TRACE(Common_Filesystem, "{} --> {}", srcFilename, destFilename);
    if (rename(srcFilename.c_str(), destFilename.c_str()) == 0)
        return true;
    LOG_ERROR(Common_Filesystem, "failed {} --> {}: {}", srcFilename, destFilename,
              GetLastErrorMsg());
    return false;
}

bool Copy(const std::string& srcFilename, const std::string& destFilename) {
    LOG_TRACE(Common_Filesystem, "{} --> {}", srcFilename, destFilename);
    using CFilePointer = std::unique_ptr<FILE, decltype(&std::fclose)>;

    // Open input file
    CFilePointer input{fopen(srcFilename.c_str(), "rb"), std::fclose};
    if (!input) {
        LOG_ERROR(Common_Filesystem, "opening input failed {} --> {}: {}", srcFilename,
                  destFilename, GetLastErrorMsg());
        return false;
    }

    // open output file
    CFilePointer output{fopen(destFilename.c_str(), "wb"), std::fclose};
    if (!output) {
        LOG_ERROR(Common_Filesystem, "opening output failed {} --> {}: {}", srcFilename,
                  destFilename, GetLastErrorMsg());
        return false;
    }

    // copy loop
    std::array<char, 1024> buffer;
    while (!feof(input.get())) {
        // read input
        std::size_t rnum = fread(buffer.data(), sizeof(char), buffer.size(), input.get());
        if (rnum != buffer.size()) {
            if (ferror(input.get()) != 0) {
                LOG_ERROR(Common_Filesystem, "failed reading from source, {} --> {}: {}",
                          srcFilename, destFilename, GetLastErrorMsg());
                return false;
            }
        }

        // write output
        std::size_t wnum = fwrite(buffer.data(), sizeof(char), rnum, output.get());
        if (wnum != rnum) {
            LOG_ERROR(Common_Filesystem, "failed writing to output, {} --> {}: {}", srcFilename,
                      destFilename, GetLastErrorMsg());
            return false;
        }
    }

    return true;
}

bool CreateEmptyFile(const std::string& filename) {
    LOG_TRACE(Common_Filesystem, "{}", filename);

    if (!FileUtil::IOFile(filename, "wb").IsOpen()) {
        LOG_ERROR(Common_Filesystem, "failed {}: {}", filename, GetLastErrorMsg());
        return false;
    }

    return true;
}

bool ForeachDirectoryEntry(u64* num_entries_out, const std::string& directory,
                           const DirectoryEntryCallable& callback) {
    LOG_TRACE(Common_Filesystem, "directory {}", directory);

    // How many files + directories we found
    u64 found_entries = 0;

    // Save the status of callback function
    bool callback_error = false;

    DIR* dirp = opendir(directory.c_str());
    if (!dirp)
        return false;

    // non windows loop
    while (struct dirent* result = readdir(dirp)) {
        const std::string virtual_name(result->d_name);

        if (virtual_name == "." || virtual_name == "..")
            continue;

        u64 ret_entries = 0;
        if (!callback(&ret_entries, directory, virtual_name)) {
            callback_error = true;
            break;
        }
        found_entries += ret_entries;
    }
    closedir(dirp);

    if (callback_error)
        return false;

    // num_entries_out is allowed to be specified nullptr, in which case we shouldn't try to set it
    if (num_entries_out != nullptr)
        *num_entries_out = found_entries;
    return true;
}

u64 ScanDirectoryTree(const std::string& directory, FSTEntry& parent_entry,
                      unsigned int recursion) {
    const auto callback = [recursion, &parent_entry](u64* num_entries_out,
                                                     const std::string& directory,
                                                     const std::string& virtual_name) -> bool {
        FSTEntry entry;
        entry.virtualName = virtual_name;
        entry.physicalName = directory + DIR_SEP + virtual_name;

        if (IsDirectory(entry.physicalName)) {
            entry.isDirectory = true;
            // is a directory, lets go inside if we didn't recurse to often
            if (recursion > 0) {
                entry.size = ScanDirectoryTree(entry.physicalName, entry, recursion - 1);
                *num_entries_out += entry.size;
            } else {
                entry.size = 0;
            }
        } else { // is a file
            entry.isDirectory = false;
            entry.size = GetSize(entry.physicalName);
        }
        (*num_entries_out)++;

        // Push into the tree
        parent_entry.children.push_back(std::move(entry));
        return true;
    };

    u64 num_entries;
    return ForeachDirectoryEntry(&num_entries, directory, callback) ? num_entries : 0;
}

void GetAllFilesFromNestedEntries(FSTEntry& directory, std::vector<FSTEntry>& output) {
    std::vector<FSTEntry> files;
    for (auto& entry : directory.children) {
        if (entry.isDirectory) {
            GetAllFilesFromNestedEntries(entry, output);
        } else {
            output.push_back(entry);
        }
    }
}

bool DeleteDirRecursively(const std::string& directory, unsigned int recursion) {
    const auto callback = [recursion](u64* num_entries_out, const std::string& directory,
                                      const std::string& virtual_name) -> bool {
        std::string new_path = directory + DIR_SEP_CHR + virtual_name;

        if (IsDirectory(new_path)) {
            if (recursion == 0)
                return false;
            return DeleteDirRecursively(new_path, recursion - 1);
        }
        return Delete(new_path);
    };

    if (!ForeachDirectoryEntry(nullptr, directory, callback))
        return false;

    // Delete the outermost directory
    FileUtil::DeleteDir(directory);
    return true;
}

void CopyDir(const std::string& source_path, const std::string& dest_path) {
    if (source_path == dest_path)
        return;
    if (!FileUtil::Exists(source_path))
        return;
    if (!FileUtil::Exists(dest_path))
        FileUtil::CreateFullPath(dest_path);

    DIR* dirp = opendir(source_path.c_str());
    if (!dirp)
        return;

    while (struct dirent* result = readdir(dirp)) {
        const std::string virtualName(result->d_name);
        // check for "." and ".."
        if (((virtualName[0] == '.') && (virtualName[1] == '\0')) ||
            ((virtualName[0] == '.') && (virtualName[1] == '.') && (virtualName[2] == '\0')))
            continue;

        std::string source, dest;
        source = source_path + virtualName;
        dest = dest_path + virtualName;
        if (IsDirectory(source)) {
            source += '/';
            dest += '/';
            if (!FileUtil::Exists(dest))
                FileUtil::CreateFullPath(dest);
            CopyDir(source, dest);
        } else if (!FileUtil::Exists(dest))
            FileUtil::Copy(source, dest);
    }
    closedir(dirp);
}

namespace {
std::unordered_map<UserPath, std::string> g_paths;
}

void SetUserPath(const std::string& path) {
    std::string& user_path = g_paths[UserPath::UserDir];

    if (!path.empty() && CreateFullPath(path)) {
        LOG_INFO(Common_Filesystem, "Using {} as the user directory", path);
        user_path = path;
        g_paths.emplace(UserPath::ConfigDir, user_path + CONFIG_DIR DIR_SEP);
        g_paths.emplace(UserPath::CacheDir, user_path + CACHE_DIR DIR_SEP);
    } else {
        if (FileUtil::Exists(ROOT_DIR DIR_SEP SDCARD_DIR)) {
            user_path = ROOT_DIR DIR_SEP SDCARD_DIR DIR_SEP EMU_DATA_DIR DIR_SEP;
            g_paths.emplace(UserPath::ConfigDir, user_path + CONFIG_DIR DIR_SEP);
            g_paths.emplace(UserPath::CacheDir, user_path + CACHE_DIR DIR_SEP);
        }
    }
    g_paths.emplace(UserPath::SDMCDir, user_path + SDMC_DIR DIR_SEP);
    g_paths.emplace(UserPath::NANDDir, user_path + NAND_DIR DIR_SEP);
    g_paths.emplace(UserPath::SysDataDir, user_path + SYSDATA_DIR DIR_SEP);
    // TODO: Put the logs in a better location for each OS
    g_paths.emplace(UserPath::LogDir, user_path + LOG_DIR DIR_SEP);
    g_paths.emplace(UserPath::CheatsDir, user_path + CHEATS_DIR DIR_SEP);
    g_paths.emplace(UserPath::DLLDir, user_path + DLL_DIR DIR_SEP);
    g_paths.emplace(UserPath::ShaderDir, user_path + SHADER_DIR DIR_SEP);
    g_paths.emplace(UserPath::DumpDir, user_path + DUMP_DIR DIR_SEP);
    g_paths.emplace(UserPath::LoadDir, user_path + LOAD_DIR DIR_SEP);
    g_paths.emplace(UserPath::StatesDir, user_path + STATES_DIR DIR_SEP);
    g_paths.emplace(UserPath::AmiiboDir, user_path + AMIIBO_DIR DIR_SEP);
}

const std::string& GetUserPath(UserPath path) {
    // Set up all paths and files on the first run
    if (g_paths.empty())
        SetUserPath("");
    return g_paths[path];
}

std::size_t WriteStringToFile(bool text_file, const std::string& filename, std::string_view str) {
    return IOFile(filename, text_file ? "w" : "wb").WriteString(str);
}

std::size_t ReadFileToString(bool text_file, const std::string& filename, std::string& str) {
    IOFile file(filename, text_file ? "r" : "rb");

    if (!file.IsOpen())
        return 0;

    str.resize(static_cast<u32>(file.GetSize()));
    return file.ReadArray(&str[0], str.size());
}

void SplitFilename83(const std::string& filename, std::array<char, 9>& short_name,
                     std::array<char, 4>& extension) {
    const std::string forbidden_characters = ".\"/\\[]:;=, ";

    // On a FAT32 partition, 8.3 names are stored as a 11 bytes array, filled with spaces.
    short_name = {{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'}};
    extension = {{' ', ' ', ' ', '\0'}};

    std::string::size_type point = filename.rfind('.');
    if (point == filename.size() - 1)
        point = filename.rfind('.', point);

    // Get short name.
    int j = 0;
    for (char letter : filename.substr(0, point)) {
        if (forbidden_characters.find(letter, 0) != std::string::npos)
            continue;
        if (j == 8) {
            // TODO(Link Mauve): also do that for filenames containing a space.
            // TODO(Link Mauve): handle multiple files having the same short name.
            short_name[6] = '~';
            short_name[7] = '1';
            break;
        }
        short_name[j++] = toupper(letter);
    }

    // Get extension.
    if (point != std::string::npos) {
        j = 0;
        for (char letter : filename.substr(point + 1, 3))
            extension[j++] = toupper(letter);
    }
}

std::vector<std::string> SplitPathComponents(std::string_view filename) {
    std::string copy(filename);
    std::replace(copy.begin(), copy.end(), '\\', '/');
    std::vector<std::string> out;

    std::stringstream stream(copy);
    std::string item;
    while (std::getline(stream, item, '/')) {
        out.push_back(std::move(item));
    }

    return out;
}

std::string_view GetParentPath(std::string_view path) {
    const auto name_bck_index = path.rfind('\\');
    const auto name_fwd_index = path.rfind('/');
    std::size_t name_index;

    if (name_bck_index == std::string_view::npos || name_fwd_index == std::string_view::npos) {
        name_index = std::min(name_bck_index, name_fwd_index);
    } else {
        name_index = std::max(name_bck_index, name_fwd_index);
    }

    return path.substr(0, name_index);
}

std::string_view GetFilename(std::string_view path) {
    const auto name_index = path.find_last_of("\\/");

    if (name_index == std::string_view::npos) {
        return {};
    }

    return path.substr(name_index + 1);
}

IOFile::IOFile(const std::string& filename, const char openmode[]) {
    Open(filename, openmode);
}

IOFile::~IOFile() {
    Close();
}

IOFile::IOFile(IOFile&& other) noexcept {
    std::swap(m_file, other.m_file);
    std::swap(m_good, other.m_good);
}

bool IOFile::Open(const std::string& filename, const char openmode[]) {
    m_file = s_io_factory->Open(filename, openmode);
    m_good = m_file != nullptr;
    return m_good;
}

void IOFile::ReadAllLines(std::vector<std::string>& lines) {
    if (!IsGood()) {
        return;
    }

    std::size_t total = GetSize();
    std::vector<char> content(total);
    std::size_t length = ReadArray(content.data(), total);
    if (length != total) {
        return;
    }

    std::size_t idx = 0;
    for (std::size_t j = 0; j < total; ++j) {
        if (content[j] == '\n') {
            lines.emplace_back(content.data() + idx, j - idx);
            idx = j + 1;
        }
    }
    if (idx < total) {
        lines.emplace_back(content.data() + idx, total - idx);
    }
}

} // namespace FileUtil
