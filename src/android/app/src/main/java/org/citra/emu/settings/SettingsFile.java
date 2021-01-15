package org.citra.emu.settings;

import android.text.TextUtils;
import android.util.Log;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.HashMap;
import java.util.Set;
import java.util.TreeSet;
import org.citra.emu.settings.model.BooleanSetting;
import org.citra.emu.settings.model.FloatSetting;
import org.citra.emu.settings.model.IntSetting;
import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.SettingSection;
import org.citra.emu.settings.model.StringSetting;
import org.citra.emu.utils.DirectoryInitialization;

public final class SettingsFile {
    // Core
    public static final String KEY_USE_CPU_JIT = "use_cpu_jit";
    public static final String KEY_IS_NEW_3DS = "is_new_3ds";
    public static final String KEY_USE_VIRTUAL_SD = "use_virtual_sd";
    public static final String KEY_SYSTEM_REGION = "region_value";
    public static final String KEY_SYSTEM_LANGUAGE = "language";
    public static final String KEY_USE_PRESENT_THREAD = "use_present_thread";
    // Renderer
    public static final String KEY_USE_GLES = "use_gles";
    public static final String KEY_SHOW_FPS = "show_fps";
    public static final String KEY_USE_HW_RENDERER = "use_hw_renderer";
    public static final String KEY_USE_HW_SHADER = "use_hw_shader";
    public static final String KEY_USE_SHADER_JIT = "use_shader_jit";
    public static final String KEY_SHADERS_ACCURATE_MUL = "accurate_mul_type";
    public static final String KEY_RESOLUTION_FACTOR = "resolution_factor";
    public static final String KEY_USE_FRAME_LIMIT = "use_frame_limit";
    public static final String KEY_FRAME_LIMIT = "frame_limit";
    public static final String KEY_FACTOR_3D = "factor_3d";
    public static final String KEY_CUSTOM_TEXTURES = "custom_textures";
    public static final String KEY_PRELOAD_TEXTURES = "preload_textures";
    public static final String KEY_LAYOUT_OPTION = "layout_option";
    public static final String KEY_SHADER_TYPE = "shader_type";
    public static final String KEY_POST_PROCESSING_SHADER = "pp_shader_name";
    // Audio
    public static final String KEY_ENABLE_DSP_LLE = "enable_dsp_lle";
    public static final String KEY_AUDIO_STRETCHING = "enable_audio_stretching";
    public static final String KEY_AUDIO_VOLUME = "volume";
    public static final String KEY_AUDIO_ENGINE = "output_engine";
    public static final String KEY_AUDIO_DEVICE = "output_device";
    // mic
    public static final String KEY_MIC_INPUT_TYPE = "mic_input_type";
    public static final String KEY_MIC_INPUT_DEVICE = "mic_input_device";
    // camera
    public static final String KEY_CAMERA_TYPE = "camera_type";
    // ocr
    public static final String KEY_BAIDU_OCR_KEY = "baidu_ocr_key";
    public static final String KEY_BAIDU_OCR_SECRET = "baidu_ocr_secret";
    // controls
    public static final String KEY_BUTTON_A = "button_a";
    public static final String KEY_BUTTON_B = "button_b";
    public static final String KEY_BUTTON_X = "button_x";
    public static final String KEY_BUTTON_Y = "button_y";
    public static final String KEY_BUTTON_UP = "button_up";
    public static final String KEY_BUTTON_DOWN = "button_down";
    public static final String KEY_BUTTON_LEFT = "button_left";
    public static final String KEY_BUTTON_RIGHT = "button_right";
    public static final String KEY_BUTTON_L = "button_l";
    public static final String KEY_BUTTON_R = "button_r";
    public static final String KEY_BUTTON_START = "button_start";
    public static final String KEY_BUTTON_SELECT = "button_select";
    public static final String KEY_BUTTON_DEBUG = "button_debug";
    public static final String KEY_BUTTON_GPIO14 = "button_gpio14";
    public static final String KEY_BUTTON_ZL = "button_zl";
    public static final String KEY_BUTTON_ZR = "button_zr";
    public static final String KEY_BUTTON_HOME = "button_home";
    public static final String KEY_CIRCLE_PAD_X = "circle_pad_x";
    public static final String KEY_CIRCLE_PAD_Y = "circle_pad_y";
    public static final String KEY_C_STICK_X = "c_stick_x";
    public static final String KEY_C_STICK_Y = "c_stick_y";

    /**
     * Reads a given .ini file from disk and returns it as a HashMap of Settings, themselves
     * effectively a HashMap of key/value settings. If unsuccessful, outputs an error telling why it
     * failed.
     */
    public static HashMap<String, SettingSection> loadSettings(String gameId) {
        HashMap<String, SettingSection> sections = new Settings.SettingsSectionMap();
        File ini = new File(DirectoryInitialization.getConfigFile());
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(ini));

            SettingSection current = null;
            for (String line; (line = reader.readLine()) != null;) {
                if (line.startsWith("[") && line.endsWith("]")) {
                    current = new SettingSection(line.substring(1, line.length() - 1));
                    sections.put(current.getName(), current);
                } else if ((current != null)) {
                    Setting setting = settingFromLine(current, line);
                    if (setting != null) {
                        current.putSetting(setting);
                    }
                }
            }
        } catch (FileNotFoundException e) {
            Log.e("citra",
                  "[SettingsFile] File not found: " + ini.getAbsolutePath() + e.getMessage());
        } catch (IOException e) {
            Log.e("citra",
                  "[SettingsFile] Error reading from: " + ini.getAbsolutePath() + e.getMessage());
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException e) {
                    Log.e("citra", "[SettingsFile] Error closing: " + ini.getAbsolutePath() +
                                       e.getMessage());
                }
            }
        }

        return sections;
    }

    /**
     * For a line of text, determines what type of data is being represented, and returns
     * a Setting object containing this data.
     *
     * @param current The section currently being parsed by the consuming method.
     * @param line    The line of text being parsed.
     * @return A typed Setting containing the key/value contained in the line.
     */
    private static Setting settingFromLine(SettingSection current, String line) {
        String[] splitLine = line.split("=");

        if (splitLine.length != 2) {
            Log.w("citra", "Skipping invalid config line \"" + line + "\"");
            return null;
        }

        String key = splitLine[0].trim();
        String value = splitLine[1].trim();

        if (value.length() > 12) {
            return new StringSetting(key, current.getName(), value);
        }

        try {
            int valueAsInt = Integer.valueOf(value);
            return new IntSetting(key, current.getName(), valueAsInt);
        } catch (NumberFormatException ex) {
            // ignore
        }

        try {
            float valueAsFloat = Float.valueOf(value);
            return new FloatSetting(key, current.getName(), valueAsFloat);
        } catch (NumberFormatException ex) {
            // ignore
        }

        switch (value) {
        case "True":
            return new BooleanSetting(key, current.getName(), true);
        case "False":
            return new BooleanSetting(key, current.getName(), false);
        default:
            return new StringSetting(key, current.getName(), value);
        }
    }

    /**
     * Saves a Settings HashMap to a given .ini file on disk. If unsuccessful, outputs an error
     * telling why it failed.
     *
     * @param sections The HashMap containing the Settings we want to serialize.
     */
    public static void saveFile(HashMap<String, SettingSection> sections) {
        File ini = new File(DirectoryInitialization.getConfigFile());
        PrintWriter writer = null;
        try {
            writer = new PrintWriter(ini, "UTF-8");

            Set<String> keySet = sections.keySet();
            Set<String> sortedKeySet = new TreeSet<>(keySet);

            for (String key : sortedKeySet) {
                SettingSection section = sections.get(key);
                writeSection(writer, section);
            }
        } catch (FileNotFoundException e) {
            Log.e("citra", "[SettingsFile] File not found: " + e.getMessage());
        } catch (UnsupportedEncodingException e) {
            Log.e("citra",
                  "[SettingsFile] Bad encoding; please file a bug report: " + e.getMessage());
        } finally {
            if (writer != null) {
                writer.close();
            }
        }
    }

    /**
     * Writes the contents of a Section HashMap to disk.
     *
     * @param writer  A PrintWriter pointed at a file on disk.
     * @param section A section containing settings to be written to the file.
     */
    private static void writeSection(PrintWriter writer, SettingSection section) {
        // Write this section's values.
        HashMap<String, Setting> settings = section.getSettings();
        if (settings.size() == 0)
            return;

        // Write the section header.
        String header = "[" + section.getName() + "]";
        writer.println(header);

        Set<String> sortedKeySet = new TreeSet<>(settings.keySet());
        for (String key : sortedKeySet) {
            Setting setting = settings.get(key);
            String valueAsString = setting.getValueAsString();
            if (!TextUtils.isEmpty(valueAsString)) {
                writer.println(setting.getKey() + " = " + valueAsString);
            }
        }
    }
}
