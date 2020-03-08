package org.citra.emu.settings.view;

import org.citra.emu.settings.model.BooleanSetting;
import org.citra.emu.settings.model.Setting;

public final class CheckBoxSetting extends SettingsItem {
    private boolean mDefaultValue;

    public CheckBoxSetting(String key, String section, int titleId, int descriptionId,
                           boolean defaultValue, Setting setting) {
        super(key, section, setting, titleId, descriptionId);
        mDefaultValue = defaultValue;
    }

    public boolean isChecked() {
        boolean value = mDefaultValue;
        if (getSetting() != null) {
            BooleanSetting setting = (BooleanSetting)getSetting();
            value = isInvertedSetting() != setting.getValue();
        }
        return value;
    }

    private boolean isInvertedSetting() {
        return false;
    }

    public BooleanSetting setChecked(boolean checked) {
        if (isInvertedSetting())
            checked = !checked;

        if (getSetting() == null) {
            BooleanSetting setting = new BooleanSetting(getKey(), getSection(), checked);
            setSetting(setting);
            return setting;
        } else {
            BooleanSetting setting = (BooleanSetting)getSetting();
            setting.setValue(checked);
            return null;
        }
    }

    @Override
    public int getType() {
        return TYPE_CHECKBOX;
    }
}
