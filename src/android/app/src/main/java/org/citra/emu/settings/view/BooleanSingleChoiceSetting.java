package org.citra.emu.settings.view;

import org.citra.emu.settings.model.BooleanSetting;
import org.citra.emu.settings.model.Setting;

public final class BooleanSingleChoiceSetting extends SettingsItem {
    private final boolean mDefaultValue;
    private final int mChoicesId;

    public BooleanSingleChoiceSetting(String key, String section, int titleId, int descriptionId,
                                      int choicesId, boolean defaultValue, Setting setting) {
        super(key, section, setting, titleId, descriptionId);
        mChoicesId = choicesId;
        mDefaultValue = defaultValue;
    }

    public int getChoicesId() {
        return mChoicesId;
    }

    public boolean isChecked() {
        if (getSetting() instanceof BooleanSetting) {
            return ((BooleanSetting)getSetting()).getValue();
        }
        return mDefaultValue;
    }

    public int getSelectedIndex() {
        return isChecked() ? 1 : 0;
    }

    public BooleanSetting setSelectedIndex(int index) {
        final boolean checked = index > 0;
        if (getSetting() == null) {
            BooleanSetting setting = new BooleanSetting(getKey(), getSection(), checked);
            setSetting(setting);
            return setting;
        }

        BooleanSetting setting = (BooleanSetting)getSetting();
        setting.setValue(checked);
        return null;
    }

    @Override
    public int getType() {
        return TYPE_SINGLE_CHOICE;
    }
}
