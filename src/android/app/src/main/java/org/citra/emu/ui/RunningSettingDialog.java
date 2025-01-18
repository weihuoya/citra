package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Debug;
import android.os.Handler;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.DialogFragment;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import androidx.preference.PreferenceManager;

import com.nononsenseapps.filepicker.DividerItemDecoration;
import java.util.ArrayList;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.overlay.InputOverlay;
import org.citra.emu.utils.NetPlayManager;

public class RunningSettingDialog extends DialogFragment {
    private static String KEY_MENU = "menu";
    public static final int MENU_MAIN = 0;
    public static final int MENU_SETTINGS = 1;
    public static final int MENU_MULTIPLAYER = 2;
    public static final int MENU_AMIIBO = 3;

    private int mMenu;
    private TextView mTitle;
    private TextView mInfo;
    private Handler mHandler;
    private SettingsAdapter mAdapter;
    private DialogInterface.OnDismissListener mDismissListener;

    public static RunningSettingDialog newInstance(int menu) {
        RunningSettingDialog dialog = new RunningSettingDialog();
        Bundle args = new Bundle();
        args.putInt(KEY_MENU, menu);
        dialog.setArguments(args);
        return dialog;
    }

    @NonNull
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        ViewGroup contents = (ViewGroup)getActivity().getLayoutInflater().inflate(
            R.layout.dialog_running_settings, null);

        mTitle = contents.findViewById(R.id.text_title);
        mInfo = contents.findViewById(R.id.text_info);
        mHandler = new Handler(getActivity().getMainLooper());
        setHeapInfo();

        Drawable lineDivider = getContext().getDrawable(R.drawable.line_divider);
        RecyclerView recyclerView = contents.findViewById(R.id.list_settings);
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(getContext());
        recyclerView.setLayoutManager(layoutManager);
        mAdapter = new SettingsAdapter();
        recyclerView.setAdapter(mAdapter);
        recyclerView.addItemDecoration(new DividerItemDecoration(lineDivider));
        builder.setView(contents);
        loadSubMenu(getArguments().getInt(KEY_MENU));
        return builder.create();
    }

    @Override
    public void onDismiss(DialogInterface dialog) {
        super.onDismiss(dialog);
        if (mMenu == MENU_SETTINGS) {
            mAdapter.saveSettings();
        }
        if (mDismissListener != null) {
            mDismissListener.onDismiss(dialog);
        }
        mHandler.removeCallbacksAndMessages(null);
    }

    public void setHeapInfo() {
        long heapsize = Debug.getNativeHeapAllocatedSize() >> 20;
        mInfo.setText(String.format("%dMB", heapsize));
        mHandler.postDelayed(this::setHeapInfo, 1000);
    }

    public void setOnDismissListener(DialogInterface.OnDismissListener listener) {
        mDismissListener = listener;
    }

    private void loadSubMenu(int menu) {
        if (menu == MENU_MAIN) {
            EmulationActivity activity = (EmulationActivity)NativeLibrary.getEmulationContext();
            mTitle.setText(activity.getGameName());
            mAdapter.loadMainMenu();
        } else if (menu == MENU_SETTINGS) {
            mTitle.setText(R.string.preferences_settings);
            mAdapter.loadSettingsMenu();
        } else if (menu == MENU_MULTIPLAYER) {
            mTitle.setText(R.string.multiplayer);
            mAdapter.loadMultiplayerMenu();
        }
        mMenu = menu;
    }

    public class SettingsItem {
        // setting type
        public static final int SETTING_CORE_TICKS_HACK = 0;
        public static final int SETTING_SKIP_SLOW_DRAW = 1;
        public static final int SETTING_SKIP_CPU_WRITE = 2;
        public static final int SETTING_SKIP_TEXTURE_COPY = 3;
        public static final int SETTING_FORCE_TEXTURE_FILTER = 4;
        public static final int SETTING_HW_GS_MODE = 5;
        public static final int SETTING_SHADOW_RENDERING = 6;
        public static final int SETTING_ASYNC_SHADER_COMPILE = 7;
        public static final int SETTING_USE_COMPATIBLE_MODE = 8;
        public static final int SETTING_SCALE_FACTOR = 9;
        public static final int SETTING_SCREEN_LAYOUT = 10;
        public static final int SETTING_ACCURATE_MUL = 11;
        public static final int SETTING_CUSTOM_LAYOUT = 12;
        public static final int SETTING_FRAME_LIMIT = 13;

        // pref
        public static final int SETTING_JOYSTICK_RELATIVE = 100;
        public static final int SETTING_HIDE_INPUT_OVERLAY = 101;
        public static final int SETTING_CONTROLLER_SCALE = 102;
        public static final int SETTING_CONTROLLER_ALPHA = 103;
        public static final int SETTING_USE_HAPTIC_FEEDBACK = 104;

        // func
        public static final int SETTING_LOAD_SUBMENU = 201;
        public static final int SETTING_EDIT_BUTTONS = 202;
        public static final int SETTING_RESET_CAMERA = 203;
        public static final int SETTING_ROTATE_SCREEN = 204;
        public static final int SETTING_CHEAT_CODE = 205;
        public static final int SETTING_MEMORY_VIEWER = 206;
        public static final int SETTING_EDIT_SCREEN = 207;
        public static final int SETTING_EXIT_GAME = 208;

        public static final int SETTING_MULTIPLAYER_ROOM_TEXT = 400;
        public static final int SETTING_MULTIPLAYER_CREATE_ROOM = 401;
        public static final int SETTING_MULTIPLAYER_JOIN_ROOM = 402;
        public static final int SETTING_MULTIPLAYER_ROOM_MEMBER = 403;
        public static final int SETTING_MULTIPLAYER_EXIT_ROOM = 404;

        // view type
        public static final int TYPE_CHECKBOX = 0;
        public static final int TYPE_RADIO_GROUP = 1;
        public static final int TYPE_SEEK_BAR = 2;
        public static final int TYPE_BUTTON = 3;
        public static final int TYPE_TEXT = 4;

        private int mSetting;
        private String mName;
        private int mType;
        private int mValue;

        public SettingsItem(int setting, int nameId, int type, int value) {
            mSetting = setting;
            mName = getString(nameId);
            mType = type;
            mValue = value;
        }

        public SettingsItem(int setting, String name, int type, int value) {
            mSetting = setting;
            mName = name;
            mType = type;
            mValue = value;
        }

        public int getType() {
            return mType;
        }

        public int getSetting() {
            return mSetting;
        }

        public String getName() {
            return mName;
        }

        public int getValue() {
            return mValue;
        }

        public void setValue(int value) {
            mValue = value;
        }
    }

    public abstract class SettingViewHolder
        extends RecyclerView.ViewHolder implements View.OnClickListener {
        public SettingViewHolder(View itemView) {
            super(itemView);
            itemView.setOnClickListener(this);
            findViews(itemView);
        }

        protected abstract void findViews(View root);

        public abstract void bind(SettingsItem item);

        public abstract void onClick(View clicked);
    }

    public final class TextSettingViewHolder extends SettingViewHolder {
        SettingsItem mItem;
        private TextView mName;

        public TextSettingViewHolder(View itemView) {
            super(itemView);
        }

        @Override
        protected void findViews(View root) {
            mName = root.findViewById(R.id.text_setting_name);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mName.setText(item.getName());
            if (mItem.getSetting() == SettingsItem.SETTING_MULTIPLAYER_ROOM_MEMBER && mItem.getValue() != 0) {
                mName.setTextColor(mName.getContext().getColor(R.color.citra_accent));
            } else {
                mName.setTextColor(mName.getContext().getColor(R.color.foreground_color));
            }
        }

        @Override
        public void onClick(View clicked) {
            EmulationActivity activity = (EmulationActivity)NativeLibrary.getEmulationContext();
            switch (mItem.getSetting()) {
            case SettingsItem.SETTING_LOAD_SUBMENU:
                loadSubMenu(mItem.getValue());
                break;
            case SettingsItem.SETTING_EDIT_BUTTONS:
                activity.startConfiguringControls();
                dismiss();
                break;
            case SettingsItem.SETTING_RESET_CAMERA:
                NativeLibrary.ResetCamera();
                dismiss();
                break;
            case SettingsItem.SETTING_ROTATE_SCREEN:
                activity.rotateScreen();
                dismiss();
                break;
            case SettingsItem.SETTING_CHEAT_CODE:
                activity.launchCheatCode();
                dismiss();
                break;
            case SettingsItem.SETTING_MEMORY_VIEWER:
                activity.launchMemoryViewer();
                dismiss();
                break;
            case SettingsItem.SETTING_EDIT_SCREEN:
                activity.startConfiguringLayout();
                dismiss();
                break;
            case SettingsItem.SETTING_EXIT_GAME:
                NativeLibrary.StopEmulation();
                dismiss();
                break;
            case SettingsItem.SETTING_MULTIPLAYER_CREATE_ROOM:
                NetPlayManager.ShowCreateRoomDialog(getActivity());
                dismiss();
                break;
            case SettingsItem.SETTING_MULTIPLAYER_JOIN_ROOM:
                NetPlayManager.ShowJoinRoomDialog(getActivity());
                dismiss();
                break;
            case SettingsItem.SETTING_MULTIPLAYER_EXIT_ROOM:
                NetPlayManager.NetPlayLeaveRoom();
                dismiss();
                break;
            }
        }
    }

    public final class ButtonSettingViewHolder extends SettingViewHolder {
        SettingsItem mItem;
        private TextView mName;
        private Button mButton;

        public ButtonSettingViewHolder(View itemView) {
            super(itemView);
            itemView.setOnClickListener(null);
        }

        @Override
        protected void findViews(View root) {
            mName = root.findViewById(R.id.text_setting_name);
            mButton = root.findViewById(R.id.button_setting);
            mButton.setText(R.string.multiplayer_kick_member);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mName.setText(mItem.getName());
            mButton.setOnClickListener(this);
        }

        @Override
        public void onClick(View clicked) {
            if (mItem.getSetting() == SettingsItem.SETTING_MULTIPLAYER_ROOM_MEMBER) {
                String text = mItem.getName();
                int pos = text.indexOf('[');
                if (pos > 0) {
                    text = text.substring(0, pos - 1);
                }
                NetPlayManager.NetPlayKickUser(text);
            }
        }
    }

    public final class CheckBoxSettingViewHolder
        extends SettingViewHolder implements CompoundButton.OnCheckedChangeListener {
        SettingsItem mItem;
        private TextView mTextSettingName;
        private CheckBox mCheckbox;

        public CheckBoxSettingViewHolder(View itemView) {
            super(itemView);
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mCheckbox = root.findViewById(R.id.checkbox);
            mCheckbox.setOnCheckedChangeListener(this);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mTextSettingName.setText(item.getName());
            mCheckbox.setChecked(mItem.getValue() > 0);
        }

        @Override
        public void onClick(View clicked) {
            mCheckbox.toggle();
            mItem.setValue(mCheckbox.isChecked() ? 1 : 0);
        }

        @Override
        public void onCheckedChanged(CompoundButton view, boolean isChecked) {
            mItem.setValue(isChecked ? 1 : 0);
        }
    }

    public final class RadioButtonSettingViewHolder
            extends SettingViewHolder implements RadioGroup.OnCheckedChangeListener
    {
        SettingsItem mItem;
        private TextView mTextSettingName;
        private RadioGroup mRadioGroup;

        public RadioButtonSettingViewHolder(View itemView)
        {
            super(itemView);
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mRadioGroup = root.findViewById(R.id.radio_group);
            mRadioGroup.setOnCheckedChangeListener(this);
        }

        @Override
        public void bind(SettingsItem item) {
            int checkIds[] = {R.id.radio0, R.id.radio1, R.id.radio2, R.id.radio3};
            int index = item.getValue();
            if (index < 0 || index >= checkIds.length)
                index = 0;

            mItem = item;
            mTextSettingName.setText(item.getName());
            mRadioGroup.check(checkIds[index]);

            if (item.getSetting() == SettingsItem.SETTING_SCREEN_LAYOUT) {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText(R.string.default_value);

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText(R.string.single_screen_option);

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText(R.string.large_screen_option);

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setVisibility(View.VISIBLE);
                radio3.setText(R.string.side_screen_option);
            } else if (item.getSetting() == SettingsItem.SETTING_SCALE_FACTOR) {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText("×1");

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText("×2");

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText("×3");

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setVisibility(View.VISIBLE);
                radio3.setText("×4");
            } else if (item.getSetting() == SettingsItem.SETTING_ACCURATE_MUL) {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText(R.string.accurate_mul_off);

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText(R.string.accurate_mul_fast);

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText(R.string.accurate_mul_accurate);

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_FORCE_TEXTURE_FILTER) {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText(R.string.auto);

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText(R.string.nearest);

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText(R.string.linear);

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_HW_GS_MODE) {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText(R.string.auto);

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText(R.string.enable);

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText(R.string.off);

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setVisibility(View.GONE);
            }
        }

        @Override
        public void onClick(View clicked) {}

        @Override
        public void onCheckedChanged(RadioGroup group, int checkedId) {
            switch (checkedId) {
                case R.id.radio0:
                    mItem.setValue(0);
                    break;
                case R.id.radio1:
                    mItem.setValue(1);
                    break;
                case R.id.radio2:
                    mItem.setValue(2);
                    break;
                case R.id.radio3:
                    mItem.setValue(3);
                    break;
                default:
                    mItem.setValue(0);
                    break;
            }
        }
    }

    public final class SeekBarSettingViewHolder extends SettingViewHolder {
        SettingsItem mItem;
        private TextView mTextSettingName;
        private TextView mTextSettingValue;
        private SeekBar mSeekBar;
        private final SeekBar.OnSeekBarChangeListener mOnSeekBarChangeListener;

        public SeekBarSettingViewHolder(View itemView) {
            super(itemView);
            mOnSeekBarChangeListener = new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    refreshProgress(progress);
                }

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {

                }

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {

                }
            };
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mTextSettingValue = root.findViewById(R.id.text_setting_value);
            mSeekBar = root.findViewById(R.id.seekbar);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mTextSettingName.setText(item.getName());
            mSeekBar.setOnSeekBarChangeListener(null);
            if (mItem.getSetting() == SettingsItem.SETTING_FRAME_LIMIT) {
                mSeekBar.setMax(200);
            } else {
                mSeekBar.setMax(100);
            }
            mSeekBar.setProgress(item.getValue());
            mSeekBar.setOnSeekBarChangeListener(mOnSeekBarChangeListener);
            refreshProgress(item.getValue());
        }

        private void refreshProgress(int progress) {
            if (mSeekBar.getMax() > 99) {
                mTextSettingValue.setText(progress + "%");
            } else {
                mTextSettingValue.setText(String.valueOf(progress));
            }
            mItem.setValue(progress);
        }

        @Override
        public void onClick(View clicked) {}
    }

    public class SettingsAdapter extends RecyclerView.Adapter<SettingViewHolder> {
        private int[] mRunningSettings;
        private int mUseHapticFeedback;
        private int mJoystickRelative;
        private int mHideInputOverlay;
        private int mControllerScale;
        private int mControllerAlpha;
        private ArrayList<SettingsItem> mSettings;

        public void loadMainMenu() {
            mSettings = new ArrayList<>();
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LOAD_SUBMENU, R.string.preferences_settings, SettingsItem.TYPE_TEXT, MENU_SETTINGS));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_EDIT_BUTTONS, R.string.emulation_edit_layout, SettingsItem.TYPE_TEXT, 0));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_RESET_CAMERA, R.string.menu_emulation_camera, SettingsItem.TYPE_TEXT, 0));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_ROTATE_SCREEN, R.string.emulation_screen_rotation, SettingsItem.TYPE_TEXT, 0));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CHEAT_CODE, R.string.menu_cheat_code, SettingsItem.TYPE_TEXT, 0));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_MEMORY_VIEWER, R.string.emulation_memory_search, SettingsItem.TYPE_TEXT, 0));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LOAD_SUBMENU, R.string.multiplayer, SettingsItem.TYPE_TEXT, MENU_MULTIPLAYER));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_EDIT_SCREEN, R.string.emulation_screen_layout, SettingsItem.TYPE_TEXT, 0));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_EXIT_GAME, R.string.emulation_stop_running, SettingsItem.TYPE_TEXT, 0));
            notifyDataSetChanged();
        }

        public void loadMultiplayerMenu() {
            String[] infos = NetPlayManager.NetPlayRoomInfo();
            mSettings = new ArrayList<>();

            if (infos.length > 0) {
                String roomTitle = getString(R.string.multiplayer_room_title, infos[0]);
                mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_ROOM_TEXT, roomTitle, SettingsItem.TYPE_TEXT, 0));
                if (false && NetPlayManager.NetPlayIsHostedRoom()) {
                    for (int i = 1; i < infos.length; ++i) {
                        mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_ROOM_MEMBER, infos[i], SettingsItem.TYPE_BUTTON, 0));
                    }
                } else {
                    for (int i = 1; i < infos.length; ++i) {
                        mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_ROOM_MEMBER, infos[i], SettingsItem.TYPE_TEXT, 0));
                    }
                }
                mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_EXIT_ROOM, R.string.multiplayer_exit_room, SettingsItem.TYPE_TEXT, 0));
            } else {
                String consoleTitle = getString(R.string.multiplayer_console_id, NetPlayManager.NetPlayGetConsoleId());
                mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_ROOM_TEXT, consoleTitle, SettingsItem.TYPE_TEXT, 0));
                mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_CREATE_ROOM, R.string.multiplayer_create_room, SettingsItem.TYPE_TEXT, 0));
                mSettings.add(new SettingsItem(SettingsItem.SETTING_MULTIPLAYER_JOIN_ROOM, R.string.multiplayer_join_room, SettingsItem.TYPE_TEXT, 0));
            }
            notifyDataSetChanged();
        }

        public void loadSettingsMenu() {
            int i = 0;
            mRunningSettings = NativeLibrary.getRunningSettings();
            mSettings = new ArrayList<>();

            // pref settings
            mUseHapticFeedback = InputOverlay.sUseHapticFeedback ? 1 : 0;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_USE_HAPTIC_FEEDBACK,
                    R.string.use_haptic_feedback,
                    SettingsItem.TYPE_CHECKBOX, mUseHapticFeedback));

            mJoystickRelative = InputOverlay.sJoystickRelative ? 1 : 0;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_JOYSTICK_RELATIVE,
                    R.string.joystick_relative_center,
                    SettingsItem.TYPE_CHECKBOX, mJoystickRelative));

            mHideInputOverlay = InputOverlay.sHideInputOverlay ? 1 : 0;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_HIDE_INPUT_OVERLAY,
                    R.string.hide_input_overlay,
                    SettingsItem.TYPE_CHECKBOX, mHideInputOverlay));

            mControllerScale = InputOverlay.sControllerScale;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CONTROLLER_SCALE,
                    R.string.controller_scale, SettingsItem.TYPE_SEEK_BAR,
                    mControllerScale));

            mControllerAlpha = InputOverlay.sControllerAlpha;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CONTROLLER_ALPHA,
                    R.string.controller_alpha, SettingsItem.TYPE_SEEK_BAR,
                    mControllerAlpha));

            // native settings
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CORE_TICKS_HACK,
                    R.string.setting_core_ticks_hack,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_SLOW_DRAW,
                    R.string.setting_skip_slow_draw,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_CPU_WRITE,
                    R.string.setting_skip_cpu_write,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_TEXTURE_COPY,
                    R.string.setting_skip_texture_copy,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_FORCE_TEXTURE_FILTER,
                    R.string.setting_force_texture_filter,
                    SettingsItem.TYPE_RADIO_GROUP, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_HW_GS_MODE,
                    R.string.setting_hw_gs_mode,
                    SettingsItem.TYPE_RADIO_GROUP, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SHADOW_RENDERING,
                    R.string.setting_shadow_rendering,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_ASYNC_SHADER_COMPILE,
                    R.string.setting_async_shader_compile,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_USE_COMPATIBLE_MODE,
                    R.string.setting_use_compatible_mode,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SCALE_FACTOR,
                    R.string.running_resolution, SettingsItem.TYPE_RADIO_GROUP,
                    mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SCREEN_LAYOUT,
                    R.string.running_layout, SettingsItem.TYPE_RADIO_GROUP,
                    mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_ACCURATE_MUL,
                    R.string.running_accurate_mul, SettingsItem.TYPE_RADIO_GROUP,
                    mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CUSTOM_LAYOUT,
                    R.string.running_custom_layout,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_FRAME_LIMIT,
                    R.string.running_frame_limit,
                    SettingsItem.TYPE_SEEK_BAR, mRunningSettings[i++]));
            notifyDataSetChanged();
        }

        @NonNull
        @Override
        public SettingViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View itemView;
            LayoutInflater inflater = LayoutInflater.from(parent.getContext());
            switch (viewType) {
            case SettingsItem.TYPE_CHECKBOX:
                itemView = inflater.inflate(R.layout.list_item_running_checkbox, parent, false);
                return new CheckBoxSettingViewHolder(itemView);
            case SettingsItem.TYPE_RADIO_GROUP:
                itemView = inflater.inflate(R.layout.list_item_running_radio4, parent, false);
                return new RadioButtonSettingViewHolder(itemView);
            case SettingsItem.TYPE_SEEK_BAR:
                itemView = inflater.inflate(R.layout.list_item_running_seekbar, parent, false);
                return new SeekBarSettingViewHolder(itemView);
            case SettingsItem.TYPE_BUTTON:
                itemView = inflater.inflate(R.layout.list_item_running_button, parent, false);
                return new ButtonSettingViewHolder(itemView);
            case SettingsItem.TYPE_TEXT:
                itemView = inflater.inflate(R.layout.list_item_running_text, parent, false);
                return new TextSettingViewHolder(itemView);
            }
            return null;
        }

        @Override
        public int getItemCount() {
            return mSettings.size();
        }

        @Override
        public int getItemViewType(int position) {
            return mSettings.get(position).getType();
        }

        @Override
        public void onBindViewHolder(@NonNull SettingViewHolder holder, int position) {
            holder.bind(mSettings.get(position));
        }

        public void saveSettings() {
            if (mRunningSettings == null) {
                return;
            }

            EmulationActivity activity = (EmulationActivity)NativeLibrary.getEmulationContext();

            // pref settings
            SharedPreferences.Editor editor =
                PreferenceManager.getDefaultSharedPreferences(activity).edit();

            int feedback = mSettings.get(0).getValue();
            if (mUseHapticFeedback != feedback) {
                editor.putBoolean(InputOverlay.PREF_HAPTIC_FEEDBACK, feedback > 0);
                InputOverlay.sUseHapticFeedback = feedback > 0;
            }
            mSettings.remove(0);

            int relative = mSettings.get(0).getValue();
            if (mJoystickRelative != relative) {
                editor.putBoolean(InputOverlay.PREF_JOYSTICK_RELATIVE, relative > 0);
                InputOverlay.sJoystickRelative = relative > 0;
            }
            mSettings.remove(0);

            int hide = mSettings.get(0).getValue();
            if (mHideInputOverlay != hide) {
                editor.putBoolean(InputOverlay.PREF_CONTROLLER_HIDE, hide > 0);
                InputOverlay.sHideInputOverlay = hide > 0;
            }
            mSettings.remove(0);

            int scale = mSettings.get(0).getValue();
            if (mControllerScale != scale) {
                editor.putInt(InputOverlay.PREF_CONTROLLER_SCALE, scale);
                InputOverlay.sControllerScale = scale;
            }
            mSettings.remove(0);

            int alpha = mSettings.get(0).getValue();
            if (mControllerAlpha != alpha) {
                editor.putInt(InputOverlay.PREF_CONTROLLER_ALPHA, alpha);
                InputOverlay.sControllerAlpha = alpha;
            }
            mSettings.remove(0);

            // applay prefs
            editor.apply();
            activity.refreshControls();

            // native settings
            boolean isChanged = false;
            int[] newSettings = new int[mRunningSettings.length];
            for (int i = 0; i < mRunningSettings.length; ++i) {
                newSettings[i] = mSettings.get(i).getValue();
                if (newSettings[i] != mRunningSettings[i]) {
                    isChanged = true;
                }
            }
            if (isChanged) {
                NativeLibrary.setRunningSettings(newSettings);
            }
            mRunningSettings = null;
        }
    }
}
