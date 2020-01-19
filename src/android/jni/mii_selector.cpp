
#include "mii_selector.h"

#include "common/file_util.h"
#include "common/string_util.h"
#include "core/file_sys/archive_extsavedata.h"
#include "core/file_sys/file_backend.h"
#include "core/hle/service/ptm/ptm.h"
#include "jni_common.h"

AndroidMiiSelector::AndroidMiiSelector() = default;

void AndroidMiiSelector::Setup(const Frontend::MiiSelectorConfig& config) {
    MiiSelector::Setup(config);
    LoadExtSaveData();
    std::vector<std::string> mii_name_list;
    for (const auto& mii : miis) {
        mii_name_list.push_back(Common::UTF16BufferToUTF8(mii.mii_name));
    }

    std::string title(config.title.empty() || config.title.at(0) == '\x0000' ? "Mii Selector"
                                                                             : config.title);
    ShowMiiSelectorDialog(config.enable_cancel_button, title, mii_name_list);
}

void AndroidMiiSelector::LoadExtSaveData() {
    miis.push_back(HLE::Applets::MiiSelector::GetStandardMiiResult().selected_mii_data);

    std::string nand_directory{FileUtil::GetUserPath(FileUtil::UserPath::NANDDir)};
    FileSys::ArchiveFactory_ExtSaveData extdata_archive_factory(nand_directory, true);

    auto archive_result = extdata_archive_factory.Open(Service::PTM::ptm_shared_extdata_id, 0);
    if (archive_result.Succeeded()) {
        auto archive = std::move(archive_result).Unwrap();

        FileSys::Path file_path = "/CFL_DB.dat";
        FileSys::Mode mode{};
        mode.read_flag.Assign(1);

        auto file_result = archive->OpenFile(file_path, mode);
        if (file_result.Succeeded()) {
            auto file = std::move(file_result).Unwrap();

            u32 saved_miis_offset = 0x8;
            // The Mii Maker has a 100 Mii limit on the 3ds
            for (int i = 0; i < 100; ++i) {
                HLE::Applets::MiiData mii;
                std::array<u8, sizeof(mii)> mii_raw;
                file->Read(saved_miis_offset, sizeof(mii), mii_raw.data());
                std::memcpy(&mii, mii_raw.data(), sizeof(mii));
                if (mii.mii_id != 0) {
                    miis.push_back(mii);
                }
                saved_miis_offset += sizeof(mii);
            }
        }
    }
}
