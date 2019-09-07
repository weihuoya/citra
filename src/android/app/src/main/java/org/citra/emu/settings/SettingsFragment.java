package org.citra.emu.settings;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v4.app.Fragment;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import com.nononsenseapps.filepicker.DividerItemDecoration;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.citra.emu.R;
import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.SettingSection;
import org.citra.emu.settings.view.CheckBoxSetting;
import org.citra.emu.settings.view.HeaderSetting;
import org.citra.emu.settings.view.InputBindingSetting;
import org.citra.emu.settings.view.SettingsItem;
import org.citra.emu.settings.view.SingleChoiceSetting;
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

        // core
        sl.add(new HeaderSetting(null, null, R.string.setting_header_core, 0));
        SettingSection coreSection = mSettings.getSection(Settings.SECTION_INI_CORE);
        Setting isNew3DS = coreSection.getSetting(SettingsFile.KEY_IS_NEW_3DS);
        Setting useVirtualSD = coreSection.getSetting(SettingsFile.KEY_USE_VIRTUAL_SD);
        Setting systemRegion = coreSection.getSetting(SettingsFile.KEY_SYSTEM_REGION);

        sl.add(new CheckBoxSetting(SettingsFile.KEY_IS_NEW_3DS, Settings.SECTION_INI_CORE,
                                   R.string.setting_is_new_3ds, 0, false, isNew3DS));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_VIRTUAL_SD, Settings.SECTION_INI_CORE,
                                   R.string.setting_use_virtual_sd, 0, true, useVirtualSD));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_SYSTEM_REGION, Settings.SECTION_INI_CORE,
                                       R.string.setting_region_value, 0,
                                       R.array.systemRegionEntries, R.array.systemRegionValues, -1,
                                       systemRegion));

        // renderer
        sl.add(new HeaderSetting(null, null, R.string.setting_header_renderer, 0));
        SettingSection rendererSection = mSettings.getSection(Settings.SECTION_INI_RENDERER);
        Setting layoutOption = rendererSection.getSetting(SettingsFile.KEY_LAYOUT_OPTION);
        Setting showFPS = rendererSection.getSetting(SettingsFile.KEY_SHOW_FPS);
        Setting resolution = rendererSection.getSetting(SettingsFile.KEY_RESOLUTION_FACTOR);
        Setting hwShader = rendererSection.getSetting(SettingsFile.KEY_USE_HW_SHADER);
        Setting accurateMul = rendererSection.getSetting(SettingsFile.KEY_SHADERS_ACCURATE_MUL);
        Setting shader = rendererSection.getSetting(SettingsFile.KEY_POST_PROCESSING_SHADER);

        sl.add(new SingleChoiceSetting(
            SettingsFile.KEY_LAYOUT_OPTION, Settings.SECTION_INI_RENDERER, R.string.layout_option,
            0, R.array.layoutOptionEntries, R.array.layoutOptionValues, 0, layoutOption));
        sl.add(new SingleChoiceSetting(SettingsFile.KEY_RESOLUTION_FACTOR,
                                       Settings.SECTION_INI_RENDERER, R.string.internal_resolution,
                                       0, R.array.internalResolutionEntries,
                                       R.array.internalResolutionValues, 1, resolution));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_SHOW_FPS, Settings.SECTION_INI_RENDERER,
                                   R.string.show_fps, 0, true, showFPS));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_USE_HW_SHADER,
                                   Settings.SECTION_INI_RENDERER,
                                   R.string.setting_hw_shader, 0, true, hwShader));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_SHADERS_ACCURATE_MUL,
                                   Settings.SECTION_INI_RENDERER,
                                   R.string.setting_shaders_accurate_mul, 0, false, accurateMul));
        // post process shaders
        String[] shaderListValues = getShaderValues();
        String[] shaderListEntries = getShaderEntries(shaderListValues);
        sl.add(new StringSingleChoiceSetting(
            SettingsFile.KEY_POST_PROCESSING_SHADER, Settings.SECTION_INI_RENDERER,
            R.string.post_processing_shader, 0, shaderListEntries, shaderListValues, "", shader));

        // audio
        sl.add(new HeaderSetting(null, null, R.string.setting_header_audio, 0));
        SettingSection audioSection = mSettings.getSection(Settings.SECTION_INI_AUDIO);
        Setting audioOutput = audioSection.getSetting(SettingsFile.KEY_AUDIO_ENGINE);
        Setting audioStretching = audioSection.getSetting(SettingsFile.KEY_AUDIO_STRETCHING);

        String[] outputEntries = getResources().getStringArray(R.array.audioOuputEntries);
        String[] outputValues = getResources().getStringArray(R.array.audioOuputValues);
        sl.add(new StringSingleChoiceSetting(
            SettingsFile.KEY_AUDIO_ENGINE, Settings.SECTION_INI_AUDIO,
            R.string.setting_audio_output, 0, outputEntries, outputValues, "auto", audioOutput));
        sl.add(new CheckBoxSetting(SettingsFile.KEY_AUDIO_STRETCHING, Settings.SECTION_INI_AUDIO,
                                   R.string.setting_audio_stretching, 0, false, audioStretching));

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

        Setting buttonL = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_L);
        Setting buttonR = bindingsSection.getSetting(SettingsFile.KEY_BUTTON_R);
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
