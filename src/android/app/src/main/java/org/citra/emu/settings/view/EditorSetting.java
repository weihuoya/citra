package org.citra.emu.settings.view;

import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.StringSetting;

public final class EditorSetting extends SettingsItem {

    public EditorSetting(String key, String section, Setting setting, int titleId, int descriptionId) {
        super(key, section, setting, titleId, descriptionId);
    }

    public String getSelectedValue() {
        if (getSetting() != null) {
            StringSetting setting = (StringSetting)getSetting();
            return setting.getValue();
        } else {
            return "";
        }
    }

    public StringSetting setSelectedValue(String selection) {
        if (getSetting() == null) {
            StringSetting setting = new StringSetting(getKey(), getSection(), selection);
            setSetting(setting);
            return setting;
        } else {
            StringSetting setting = (StringSetting)getSetting();
            setting.setValue(selection);
            return null;
        }
    }

    @Override
    public int getType() {
        return TYPE_EDITOR;
    }
}
