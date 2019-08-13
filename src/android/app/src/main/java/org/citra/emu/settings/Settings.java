package org.citra.emu.settings;

import org.citra.emu.settings.model.SettingSection;

import java.util.HashMap;

public final class Settings {

    public static final String SECTION_INI_CORE = "Core";
    public static final String SECTION_INI_RENDERER = "Renderer";
    public static final String SECTION_INI_AUDIO = "Audio";
    private HashMap<String, SettingSection> mSections = new SettingsSectionMap();

    public SettingSection getSection(String sectionName) {
        return mSections.get(sectionName);
    }

    public boolean isEmpty() {
        return mSections.isEmpty();
    }

    public void loadSettings(String gameId) {
        mSections = new SettingsSectionMap();
        mSections.putAll(SettingsFile.loadSettings(gameId));
    }

    public void saveSettings() {
        SettingsFile.saveFile(mSections);
    }

    /**
     * A HashMap<String, SettingSection> that constructs a new SettingSection instead of returning null
     * when getting a key not already in the map
     */
    public static final class SettingsSectionMap extends HashMap<String, SettingSection> {
        @Override
        public SettingSection get(Object key) {
            if (!(key instanceof String)) {
                return null;
            }

            String stringKey = (String) key;

            if (!super.containsKey(stringKey)) {
                SettingSection section = new SettingSection(stringKey);
                super.put(stringKey, section);
                return section;
            }
            return super.get(key);
        }
    }
}
