package org.citra.emu.settings;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.nononsenseapps.filepicker.DividerItemDecoration;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.citra.emu.R;
import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.SettingSection;
import org.citra.emu.settings.view.CheckBoxSetting;
import org.citra.emu.settings.view.EditorSetting;
import org.citra.emu.settings.view.HeaderSetting;
import org.citra.emu.settings.view.InputBindingSetting;
import org.citra.emu.settings.view.SettingsItem;
import org.citra.emu.settings.view.SingleChoiceSetting;
import org.citra.emu.settings.view.SliderSetting;
import org.citra.emu.settings.view.StringSingleChoiceSetting;
import org.citra.emu.utils.DirectoryInitialization;

public final class SettingsFragment extends Fragment {
    private static final String ARGUMENT_MENU_TAG = "menu_tag";
    private static final String ARGUMENT_GAME_ID = "game_id";

    private ArrayList<SettingsItem> mSettingsList;
    private SettingsActivity mActivity;
    private SettingsAdapter mAdapter;
    private MenuTag mMenuTag;
    private String mGameID;
    private Settings mSettings;

    public static Fragment newInstance(MenuTag menuTag, String gameId, Bundle extras) {
        SettingsFragment fragment = new SettingsFragment();

        Bundle arguments = new Bundle();
        if (extras != null) {
            arguments.putAll(extras);
        }

        arguments.putSerializable(ARGUMENT_MENU_TAG, menuTag);
        arguments.putString(ARGUMENT_GAME_ID, gameId);

        fragment.setArguments(arguments);
        return fragment;
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        mActivity = (SettingsActivity)context;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setRetainInstance(true);
        Bundle args = getArguments();
        mMenuTag = (MenuTag)args.getSerializable(ARGUMENT_MENU_TAG);
        mGameID = getArguments().getString(ARGUMENT_GAME_ID);
        mAdapter = new SettingsAdapter(mActivity);
    }

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_settings, container, false);
    }

    @Override
    public void onViewCreated(View view, @Nullable Bundle savedInstanceState) {
        Drawable lineDivider = mActivity.getDrawable(R.drawable.line_divider);
        RecyclerView recyclerView = view.findViewById(R.id.list_settings);
        recyclerView.setLayoutManager(new LinearLayoutManager(getContext()));
        recyclerView.addItemDecoration(new DividerItemDecoration(lineDivider));
        recyclerView.setAdapter(mAdapter);
        showSettingsList(mActivity.getSettings());
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mActivity = null;
    }

    public void showSettingsList(Settings settings) {
        mSettings = settings;
        if (mMenuTag == MenuTag.INPUT) {
            mSettingsList = loadBindingsList();
        } else {
            mSettingsList = loadSettingsList();
        }
        if (mSettingsList != null) {
            mAdapter.setSettings(mSettingsList);
        }
    }

    private ArrayList<SettingsItem> loadSettingsList() {
        ArrayList<SettingsItem> sl = new ArrayList<>();

        // renderer
        sl.add(new HeaderSetting(null, null, R.string.setting_header_renderer, 0));
        SettingSection rendererSection = mSettings.getSection(Settings.SECTION_INI_RENDERER);
        Setting layoutOption = rendererSection.getSetting(SettingsFile.KEY_LAYOUT_OPTION);
        Setting showFPS = rendererSection.getSetting(SettingsFile.KEY_SHOW_FPS);
        Setting resolution = rendererSection.getSetting(SettingsFile.KEY_RESOLUTION_FACTOR);
        Setting hwShader = rendererSection.getSetting(SettingsFile.KEY_USE_HW_SHADER);
        Setting accurateMul = rendererSection.getSetting(SettingsFile.KEY_SHADERS_ACCURATE_MUL);
        Setting shader = rendererSection.getSetting(SettingsFile.KEY_POST_PROCESSING_SHADER);
        Setting texLoadHack = rendererSection.getSetting(SettingsFile.KEY_TEXTURE_LOAD_HACK);
        Setting useFrameLimit = rendererSection.getSetting(SettingsFile.KEY_USE_FRAME_LIMIT);
        Setting frameLimit = rendererSection.getSetting(SettingsFile.KEY_FRAME_LIMIT);
        Setting customTex = rendererSection.getSetting(SettingsFile.KEY_CUSTOM_TEXTURES);
        Setting preloadTex = rendererSection.getSetting(SettingsFile.KEY_PRELOAD_TEXTURES);
        Setting factor3d = rendererSection.getSetting(SettingsFile.KEY_FACTOR_3D);

        SettingSection debugSection = mSettings.getSection(Settings.SECTION_INI_DEBUG);
        Setting shaderType = debugSection.getSetting(SettingsFile.KEY_SHADER_TYPE);
        Setting presentThread = debugSection.getSetting(SettingsFile.KEY_USE_PRESENT_THREAD);
        Setting ocrKey = debugSection.getSetting(SettingsFile.KEY_BAIDU_OCR_KEY);
        Setting ocrSecret = debugSection.getSetting(SettingsFile.KEY_BAIDU_OCR_SECRET);

        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_PRESENT_THREAD, Settings.SECTION_INI_DEBUG,
                R.string.setting_use_present_thread, R.string.setting_use_present_thread_desc, true, presentThread));
        sl.add(new SingleChoiceSetting(
            SettingsFile.KEY_LAYOUT_OPTION, Settings.SECTION_INI_RENDERER, R.string.layout_option,
            0, R.array.layoutOptionEntries, R.array.layoutOptionValues, 0, layoutOption));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_RESOLUTION_FACTOR,
                                       Settings.SECTION_INI_RENDERER, R.string.internal_resolution,
                                       0, R.array.internalResolutionEntries,
                                       R.array.internalResolutionValues, 1, resolution));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_SHOW_FPS, Settings.SECTION_INI_RENDERER,
                                   R.string.show_fps, 0, true, showFPS));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_HW_SHADER, Settings.SECTION_INI_RENDERER,
                                   R.string.setting_hw_shader, R.string.setting_hw_shader_desc, true, hwShader));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_SHADERS_ACCURATE_MUL,
                Settings.SECTION_INI_RENDERER, R.string.setting_shaders_accurate_mul,
                R.string.setting_shaders_accurate_mul_desc, R.array.accurateMulEntries,
                R.array.accurateMulValues, 0, accurateMul));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_SHADER_TYPE, Settings.SECTION_INI_DEBUG,
                R.string.setting_shader_type, 0, R.array.shaderEntries,
                R.array.shaderValues, 1, shaderType));
        // post process shaders
        String[] stringValues = getShaderValues();
        String[] stringEntries = getShaderEntries(stringValues);
        sl.add(new StringSingleChoiceSetting(
            SettingsFile.KEY_POST_PROCESSING_SHADER, Settings.SECTION_INI_RENDERER,
            R.string.post_processing_shader, 0, stringEntries, stringValues, "", shader));

        // custom textures
        sl.add(new CheckBoxSetting(SettingsFile.KEY_TEXTURE_LOAD_HACK,
                                   Settings.SECTION_INI_RENDERER,
                                   R.string.setting_texture_load_hack, R.string.setting_texture_load_hack_desc, false, texLoadHack));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_CUSTOM_TEXTURES, Settings.SECTION_INI_RENDERER,
                                   R.string.setting_custom_textures, 0, false, customTex));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_PRELOAD_TEXTURES, Settings.SECTION_INI_RENDERER,
                                   R.string.setting_preload_textures, 0, false, preloadTex));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_FRAME_LIMIT, Settings.SECTION_INI_RENDERER,
                R.string.frame_limit_enable, R.string.frame_limit_enable_description, true, useFrameLimit));
        sl.add(new SliderSetting(SettingsFile.KEY_FRAME_LIMIT, Settings.SECTION_INI_RENDERER,
                R.string.frame_limit_slider, R.string.frame_limit_slider_description, 200, "",
                100, frameLimit));
        sl.add(new SliderSetting(SettingsFile.KEY_FACTOR_3D, Settings.SECTION_INI_RENDERER,
                R.string.setting_factor_3d, 0, 10, "",
                0, factor3d));

        // core
        sl.add(new HeaderSetting(null, null, R.string.setting_header_core, 0));
        SettingSection coreSection = mSettings.getSection(Settings.SECTION_INI_CORE);
        Setting isNew3DS = coreSection.getSetting(SettingsFile.KEY_IS_NEW_3DS);
        Setting useVirtualSD = coreSection.getSetting(SettingsFile.KEY_USE_VIRTUAL_SD);
        Setting systemRegion = coreSection.getSetting(SettingsFile.KEY_SYSTEM_REGION);
        Setting cpuJIT = coreSection.getSetting(SettingsFile.KEY_USE_CPU_JIT);
        Setting language = coreSection.getSetting(SettingsFile.KEY_SYSTEM_LANGUAGE);

        sl.add(new CheckBoxSetting(SettingsFile.KEY_IS_NEW_3DS, Settings.SECTION_INI_CORE,
                R.string.setting_is_new_3ds, R.string.setting_is_new_3ds_desc, false, isNew3DS));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_VIRTUAL_SD, Settings.SECTION_INI_CORE,
                R.string.setting_use_virtual_sd, 0, true, useVirtualSD));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_CPU_JIT, Settings.SECTION_INI_CORE,
                R.string.setting_enable_cpu_jit, 0, true, cpuJIT));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_SYSTEM_REGION, Settings.SECTION_INI_CORE,
                R.string.setting_region_value, 0,
                R.array.systemRegionEntries, R.array.systemRegionValues, -1,
                systemRegion));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_SYSTEM_LANGUAGE, Settings.SECTION_INI_CORE,
                R.string.setting_system_language, 0,
                R.array.languageNames, R.array.languageValues, 1,
                language));

        // audio
        sl.add(new HeaderSetting(null, null, R.string.setting_header_audio, 0));
        SettingSection audioSection = mSettings.getSection(Settings.SECTION_INI_AUDIO);
        Setting audioOutput = audioSection.getSetting(SettingsFile.KEY_AUDIO_ENGINE);
        Setting audioStretching = audioSection.getSetting(SettingsFile.KEY_AUDIO_STRETCHING);

        stringEntries = getResources().getStringArray(R.array.audioOuputEntries);
        stringValues = getResources().getStringArray(R.array.audioOuputValues);
        sl.add(new StringSingleChoiceSetting(
            SettingsFile.KEY_AUDIO_ENGINE, Settings.SECTION_INI_AUDIO,
            R.string.setting_audio_output, 0, stringEntries, stringValues, "auto", audioOutput));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_AUDIO_STRETCHING, Settings.SECTION_INI_AUDIO,
                                   R.string.setting_audio_stretching, 0, false, audioStretching));

        // mic
        Setting micType = audioSection.getSetting(SettingsFile.KEY_MIC_INPUT_TYPE);
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_MIC_INPUT_TYPE, Settings.SECTION_INI_AUDIO,
                                       R.string.setting_audio_mic_type, 0, R.array.micInputEntries,
                                       R.array.micInputValues, 0, micType));
        // camera
        sl.add(new HeaderSetting(null, null, R.string.setting_header_camera, 0));
        SettingSection cameraSection = mSettings.getSection(Settings.SECTION_INI_CAMERA);
        Setting cameraType = cameraSection.getSetting(SettingsFile.KEY_CAMERA_TYPE);
        stringEntries = getResources().getStringArray(R.array.cameraEntries);
        stringValues = getResources().getStringArray(R.array.cameraValues);
        sl.add(new StringSingleChoiceSetting(
                SettingsFile.KEY_CAMERA_TYPE, Settings.SECTION_INI_CAMERA,
                R.string.setting_camera_type, 0, stringEntries, stringValues, "blank", cameraType));

        String lan = Locale.getDefault().getLanguage();
        if (lan.equals("zh")) {
            sl.add(new HeaderSetting(null, null, R.string.translate_settings, 0));
            sl.add(new EditorSetting(SettingsFile.KEY_BAIDU_OCR_KEY, Settings.SECTION_INI_DEBUG, ocrKey, R.string.baidu_ocr_key, 0));
            sl.add(new EditorSetting(SettingsFile.KEY_BAIDU_OCR_SECRET, Settings.SECTION_INI_DEBUG, ocrSecret, R.string.baidu_ocr_secret, 0));
        }

        return sl;
    }

    private ArrayList<SettingsItem> loadBindingsList() {
        ArrayList<SettingsItem> sl = new ArrayList<>();

        // controls
        SettingSection bindingsSection = mSettings.getSection(Settings.SECTION_INI_CONTROLS);
        Setting buttonA = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_A);
        Setting buttonB = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_B);
        Setting buttonX = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_X);
        Setting buttonY = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_Y);

        Setting buttonUp = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_UP);
        Setting buttonDown = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_DOWN);
        Setting buttonLeft = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_LEFT);
        Setting buttonRight = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_RIGHT);

        Setting stickUp = bindingsSection.getSetting(SettingsFile.KEY_CIRCLE_PAD_UP);
        Setting stickDown = bindingsSection.getSetting(SettingsFile.KEY_CIRCLE_PAD_DOWN);
        Setting stickLeft = bindingsSection.getSetting(SettingsFile.KEY_CIRCLE_PAD_LEFT);
        Setting stickRight = bindingsSection.getSetting(SettingsFile.KEY_CIRCLE_PAD_RIGHT);

        Setting cstickUp = bindingsSection.getSetting(SettingsFile.KEY_C_STICK_UP);
        Setting cstickDown = bindingsSection.getSetting(SettingsFile.KEY_C_STICK_DOWN);
        Setting cstickLeft = bindingsSection.getSetting(SettingsFile.KEY_C_STICK_LEFT);
        Setting cstickRight = bindingsSection.getSetting(SettingsFile.KEY_C_STICK_RIGHT);

        Setting buttonL = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_L);
        Setting buttonR = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_R);
        Setting buttonZL = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_ZL);
        Setting buttonZR = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_ZR);
        Setting buttonStart = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_START);
        Setting buttonSelect = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_SELECT);

        sl.add(new HeaderSetting(null, null, R.string.generic_buttons, 0));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_A, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_a, buttonA));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_B, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_b, buttonB));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_X, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_x, buttonX));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_Y, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_y, buttonY));

        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_L, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_l, buttonL));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_R, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_r, buttonR));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_ZL, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_zl, buttonZL));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_ZR, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_zr, buttonZR));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_START, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_start, buttonStart));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_SELECT,
                                       Settings.SECTION_INI_CONTROLS, R.string.button_select,
                                       buttonSelect));

        sl.add(new HeaderSetting(null, null, R.string.controller_dpad, 0));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_UP, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_up, buttonUp));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_DOWN, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_down, buttonDown));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_LEFT, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_left, buttonLeft));
        sl.add(new InputBindingSetting(SettingsFile.KEY_BUTTON_RIGHT, Settings.SECTION_INI_CONTROLS,
                                       R.string.button_right, buttonRight));

        sl.add(new HeaderSetting(null, null, R.string.controller_stick, 0));
        sl.add(new InputBindingSetting(SettingsFile.KEY_CIRCLE_PAD_UP, Settings.SECTION_INI_CONTROLS,
                R.string.button_up, stickUp));
        sl.add(new InputBindingSetting(SettingsFile.KEY_CIRCLE_PAD_DOWN, Settings.SECTION_INI_CONTROLS,
                R.string.button_down, stickDown));
        sl.add(new InputBindingSetting(SettingsFile.KEY_CIRCLE_PAD_LEFT, Settings.SECTION_INI_CONTROLS,
                R.string.button_left, stickLeft));
        sl.add(new InputBindingSetting(SettingsFile.KEY_CIRCLE_PAD_RIGHT, Settings.SECTION_INI_CONTROLS,
                R.string.button_right, stickRight));

        sl.add(new HeaderSetting(null, null, R.string.c_controller_stick, 0));
        sl.add(new InputBindingSetting(SettingsFile.KEY_C_STICK_UP, Settings.SECTION_INI_CONTROLS,
                R.string.button_up, cstickUp));
        sl.add(new InputBindingSetting(SettingsFile.KEY_C_STICK_DOWN, Settings.SECTION_INI_CONTROLS,
                R.string.button_down, cstickDown));
        sl.add(new InputBindingSetting(SettingsFile.KEY_C_STICK_LEFT, Settings.SECTION_INI_CONTROLS,
                R.string.button_left, cstickLeft));
        sl.add(new InputBindingSetting(SettingsFile.KEY_C_STICK_RIGHT, Settings.SECTION_INI_CONTROLS,
                R.string.button_right, cstickRight));

        return sl;
    }

    private String capitalize(String text) {
        if (text.contains("_")) {
            text = text.replace("_", " ");
        }

        if (text.length() > 1 && text.contains(" ")) {
            String[] ss = text.split(" ");
            text = capitalize(ss[0]);
            for (int i = 1; i < ss.length; ++i) {
                text += " " + capitalize(ss[i]);
            }
            return text;
        }

        return text.substring(0, 1).toUpperCase() + text.substring(1);
    }

    private String[] getShaderEntries(String[] values) {
        String[] entries = new String[values.length];
        entries[0] = mActivity.getString(R.string.off);
        for (int i = 1; i < values.length; ++i) {
            entries[i] = capitalize(values[i]);
        }
        return entries;
    }

    private String[] getShaderValues() {
        List<String> values = new ArrayList<>();
        values.add("");

        String shadersPath = DirectoryInitialization.getShadersDirectory();
        File file = new File(shadersPath);
        File[] shaderFiles = file.listFiles();
        if (shaderFiles != null) {
            for (int i = 0; i < shaderFiles.length; ++i) {
                String name = shaderFiles[i].getName();
                int extensionIndex = name.indexOf(".glsl");
                if (extensionIndex > 0) {
                    values.add(name.substring(0, extensionIndex));
                }
            }
        }

        return values.toArray(new String[0]);
    }
}
