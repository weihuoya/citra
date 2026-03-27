package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.util.DisplayMetrics;
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
    public static final int MENU_SCREEN_LAYOUT = 4;

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
        if (mMenu == MENU_SETTINGS || mMenu == MENU_SCREEN_LAYOUT) {
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
        } else if (menu == MENU_SCREEN_LAYOUT) {
            mTitle.setText(R.string.setting_header_screen_layout);
            mAdapter.loadScreenLayoutMenu();
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
        public static final int SETTING_SWAP_SCREEN = 11;
        public static final int SETTING_LARGE_SCREEN_AUTO_FIT = 12;
        public static final int SETTING_LARGE_SCREEN_PROPORTION = 13;
        public static final int SETTING_LARGE_SCREEN_SECONDARY_LEFT = 14;
        public static final int SETTING_LARGE_SCREEN_SECONDARY_TOP = 15;
        public static final int SETTING_HYBRID_SIDE_COLUMN_LEFT = 16;
        public static final int SETTING_HYBRID_SECONDARY_TOP = 17;
        // Keep these indices aligned with NativeLibrary.getRunningSettings().
        public static final int SETTING_DYNAMIC_SCREEN_FILL = 18;
        public static final int SETTING_LAYOUT_MARGIN_LEFT = 19;
        public static final int SETTING_LAYOUT_MARGIN_TOP = 20;
        public static final int SETTING_LAYOUT_MARGIN_RIGHT = 21;
        public static final int SETTING_LAYOUT_MARGIN_BOTTOM = 22;
        public static final int SETTING_ACCURATE_MUL = 23;
        public static final int SETTING_CUSTOM_LAYOUT = 24;
        public static final int SETTING_FRAME_LIMIT = 25;

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
        private SettingsItem mItem;
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
        }

        @Override
        public void onClick(View clicked) {
            if (mItem == null) {
                return;
            }

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
                NetPlayManager.ShowCreateRoomDialog(activity);
                break;
            case SettingsItem.SETTING_MULTIPLAYER_JOIN_ROOM:
                NetPlayManager.ShowJoinRoomDialog(activity);
                break;
            case SettingsItem.SETTING_MULTIPLAYER_EXIT_ROOM:
                NetPlayManager.NetPlayLeaveRoom();
                loadSubMenu(MENU_MULTIPLAYER);
                break;
            default:
                break;
            }
        }
    }

    public final class CheckBoxSettingViewHolder extends SettingViewHolder {
        private SettingsItem mItem;
        private TextView mTextSettingName;
        private CheckBox mCheckBox;

        public CheckBoxSettingViewHolder(View itemView) {
            super(itemView);
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mCheckBox = root.findViewById(R.id.checkbox);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mTextSettingName.setText(item.getName());
            mCheckBox.setChecked(item.getValue() > 0);
        }

        @Override
        public void onClick(View clicked) {
            if (mItem == null) {
                return;
            }

            final boolean checked = !mCheckBox.isChecked();
            mCheckBox.setChecked(checked);
            mItem.setValue(checked ? 1 : 0);
            mAdapter.updateWorkingValue(mItem.getSetting(), mItem.getValue());
        }
    }

    public final class RadioButtonSettingViewHolder extends SettingViewHolder
        implements RadioGroup.OnCheckedChangeListener {
        private SettingsItem mItem;
        private TextView mTextSettingName;
        private RadioGroup mRadioGroup;

        public RadioButtonSettingViewHolder(View itemView) {
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
            mItem = item;
            mTextSettingName.setText(item.getName());

            final RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
            final RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
            final RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
            final RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
            final RadioButton radio4 = mRadioGroup.findViewById(R.id.radio4);
            final RadioButton radio5 = mRadioGroup.findViewById(R.id.radio5);

            if (item.getSetting() == SettingsItem.SETTING_SCREEN_LAYOUT) {
                configureRadioButton(radio0, View.VISIBLE, R.string.default_value);
                configureRadioButton(radio1, View.VISIBLE, R.string.single_screen_option);
                configureRadioButton(radio2, View.VISIBLE, R.string.large_screen_option);
                configureRadioButton(radio3, View.VISIBLE, R.string.side_screen_option);
                configureRadioButton(radio4, View.VISIBLE, R.string.hybrid_screen_option);
                radio5.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_SCALE_FACTOR) {
                radio0.setVisibility(View.VISIBLE);
                radio0.setText("x1");
                radio1.setVisibility(View.VISIBLE);
                radio1.setText("x2");
                radio2.setVisibility(View.VISIBLE);
                radio2.setText("x3");
                radio3.setVisibility(View.VISIBLE);
                radio3.setText("x4");
                radio4.setVisibility(View.GONE);
                radio5.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_ACCURATE_MUL) {
                configureRadioButton(radio0, View.VISIBLE, R.string.accurate_mul_off);
                configureRadioButton(radio1, View.VISIBLE, R.string.accurate_mul_fast);
                configureRadioButton(radio2, View.VISIBLE, R.string.accurate_mul_accurate);
                radio3.setVisibility(View.GONE);
                radio4.setVisibility(View.GONE);
                radio5.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_FORCE_TEXTURE_FILTER) {
                configureRadioButton(radio0, View.VISIBLE, R.string.auto);
                configureRadioButton(radio1, View.VISIBLE, R.string.nearest);
                configureRadioButton(radio2, View.VISIBLE, R.string.linear);
                radio3.setVisibility(View.GONE);
                radio4.setVisibility(View.GONE);
                radio5.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_HW_GS_MODE) {
                configureRadioButton(radio0, View.VISIBLE, R.string.auto);
                configureRadioButton(radio1, View.VISIBLE, R.string.enable);
                configureRadioButton(radio2, View.VISIBLE, R.string.off);
                radio3.setVisibility(View.GONE);
                radio4.setVisibility(View.GONE);
                radio5.setVisibility(View.GONE);
            } else if (item.getSetting() == SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_LEFT ||
                       item.getSetting() == SettingsItem.SETTING_HYBRID_SIDE_COLUMN_LEFT) {
                configureRadioButton(radio0, View.VISIBLE, R.string.right);
                configureRadioButton(radio1, View.VISIBLE, R.string.left);
                hideExtraRadioButtons(radio2, radio3, radio4, radio5);
            } else if (item.getSetting() == SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_TOP ||
                       item.getSetting() == SettingsItem.SETTING_HYBRID_SECONDARY_TOP) {
                configureRadioButton(radio0, View.VISIBLE, R.string.bottom);
                configureRadioButton(radio1, View.VISIBLE, R.string.top);
                hideExtraRadioButtons(radio2, radio3, radio4, radio5);
            }

            mRadioGroup.setOnCheckedChangeListener(null);
            mRadioGroup.check(getCheckedId(item));
            mRadioGroup.setOnCheckedChangeListener(this);
        }

        @Override
        public void onClick(View clicked) {}

        @Override
        public void onCheckedChanged(RadioGroup group, int checkedId) {
            if (mItem == null) {
                return;
            }

            if (mItem.getSetting() == SettingsItem.SETTING_SCREEN_LAYOUT) {
                if (checkedId == R.id.radio0) {
                    mItem.setValue(0);
                } else if (checkedId == R.id.radio1) {
                    mItem.setValue(1);
                } else if (checkedId == R.id.radio2) {
                    mItem.setValue(2);
                } else if (checkedId == R.id.radio3) {
                    mItem.setValue(3);
                } else if (checkedId == R.id.radio4) {
                    mItem.setValue(5);
                } else {
                    mItem.setValue(0);
                }
                mAdapter.updateWorkingValue(mItem.getSetting(), mItem.getValue());
                mAdapter.loadScreenLayoutMenu();
                return;
            }

            if (checkedId == R.id.radio0) {
                mItem.setValue(0);
            } else if (checkedId == R.id.radio1) {
                mItem.setValue(1);
            } else if (checkedId == R.id.radio2) {
                mItem.setValue(2);
            } else if (checkedId == R.id.radio3) {
                mItem.setValue(3);
            } else if (checkedId == R.id.radio4) {
                mItem.setValue(4);
            } else {
                mItem.setValue(0);
            }
            mAdapter.updateWorkingValue(mItem.getSetting(), mItem.getValue());
        }

        private void configureRadioButton(RadioButton button, int visibility, int textId) {
            button.setVisibility(visibility);
            button.setText(textId);
        }

        private void hideExtraRadioButtons(RadioButton... buttons) {
            for (RadioButton button : buttons) {
                button.setVisibility(View.GONE);
            }
        }

        private int getCheckedId(SettingsItem item) {
            if (item.getSetting() == SettingsItem.SETTING_SCREEN_LAYOUT) {
                switch (item.getValue()) {
                case 0:
                    return R.id.radio0;
                case 1:
                    return R.id.radio1;
                case 2:
                case 4:
                    return R.id.radio2;
                case 3:
                    return R.id.radio3;
                case 5:
                    return R.id.radio4;
                default:
                    return R.id.radio0;
                }
            }

            final int index = Math.max(0, Math.min(item.getValue(), 4));
            return getRadioIdForIndex(index);
        }

        private int getRadioIdForIndex(int index) {
            if (index <= 0) {
                return R.id.radio0;
            }
            if (index == 1) {
                return R.id.radio1;
            }
            if (index == 2) {
                return R.id.radio2;
            }
            if (index == 3) {
                return R.id.radio3;
            }
            return R.id.radio4;
        }
    }

    public final class SeekBarSettingViewHolder extends SettingViewHolder {
        private SettingsItem mItem;
        private TextView mTextSettingName;
        private TextView mTextSettingValue;
        private SeekBar mSeekBar;
        private Button mQuickActionButton;
        private final SeekBar.OnSeekBarChangeListener mOnSeekBarChangeListener;

        public SeekBarSettingViewHolder(View itemView) {
            super(itemView);
            mOnSeekBarChangeListener = new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    refreshProgress(progress);
                }

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {}
            };
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mTextSettingValue = root.findViewById(R.id.text_setting_value);
            mSeekBar = root.findViewById(R.id.seekbar);
            mQuickActionButton = root.findViewById(R.id.button_quick_action);
            mQuickActionButton.setOnClickListener(this);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mTextSettingName.setText(item.getName());
            mSeekBar.setOnSeekBarChangeListener(null);

            switch (item.getSetting()) {
            case SettingsItem.SETTING_CONTROLLER_SCALE:
            case SettingsItem.SETTING_CONTROLLER_ALPHA:
                mQuickActionButton.setVisibility(View.GONE);
                mSeekBar.setMax(100);
                mSeekBar.setProgress(clamp(item.getValue(), 0, 100));
                break;
            case SettingsItem.SETTING_LARGE_SCREEN_PROPORTION:
                mQuickActionButton.setVisibility(View.VISIBLE);
                mSeekBar.setMax(75);
                mSeekBar.setProgress(clamp(item.getValue(), 25, 100) - 25);
                break;
            case SettingsItem.SETTING_FRAME_LIMIT:
                mQuickActionButton.setVisibility(View.GONE);
                mSeekBar.setMax(199);
                mSeekBar.setProgress(clamp(item.getValue(), 1, 200) - 1);
                break;
            case SettingsItem.SETTING_LAYOUT_MARGIN_LEFT:
            case SettingsItem.SETTING_LAYOUT_MARGIN_TOP:
            case SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT:
            case SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM:
                mQuickActionButton.setVisibility(View.GONE);
                mSeekBar.setMax(500);
                mSeekBar.setProgress(clamp(item.getValue(), 0, 500));
                break;
            default:
                mQuickActionButton.setVisibility(View.GONE);
                mSeekBar.setMax(100);
                mSeekBar.setProgress(clamp(item.getValue(), 0, 100));
                break;
            }

            refreshProgress(mSeekBar.getProgress());
            mSeekBar.setOnSeekBarChangeListener(mOnSeekBarChangeListener);
        }

        @Override
        public void onClick(View clicked) {
            if (mItem == null || clicked != mQuickActionButton) {
                return;
            }

            if (mItem.getSetting() == SettingsItem.SETTING_LARGE_SCREEN_PROPORTION) {
                final int autoFitValue = clamp(mAdapter.getLargeScreenTopAutoFitForCurrentDisplay(),
                                               25, 100);
                mSeekBar.setProgress(autoFitValue - 25);
            }
        }

        private void refreshProgress(int progress) {
            if (mItem == null) {
                return;
            }

            switch (mItem.getSetting()) {
            case SettingsItem.SETTING_CONTROLLER_SCALE:
            case SettingsItem.SETTING_CONTROLLER_ALPHA:
                mItem.setValue(progress);
                mTextSettingValue.setText(progress + "%");
                break;
            case SettingsItem.SETTING_LARGE_SCREEN_PROPORTION:
                mItem.setValue(progress + 25);
                mTextSettingValue.setText((progress + 25) + "%");
                break;
            case SettingsItem.SETTING_FRAME_LIMIT:
                mItem.setValue(progress + 1);
                mTextSettingValue.setText((progress + 1) + "%");
                break;
            case SettingsItem.SETTING_LAYOUT_MARGIN_LEFT:
            case SettingsItem.SETTING_LAYOUT_MARGIN_TOP:
            case SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT:
            case SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM:
                mItem.setValue(progress);
                mTextSettingValue.setText(progress + "px");
                break;
            default:
                mItem.setValue(progress);
                mTextSettingValue.setText(String.valueOf(progress));
                break;
            }
            mAdapter.updateWorkingValue(mItem.getSetting(), mItem.getValue());
        }

        private int clamp(int value, int min, int max) {
            return Math.max(min, Math.min(value, max));
        }
    }

    public final class ButtonSettingViewHolder extends SettingViewHolder {
        private SettingsItem mItem;
        private TextView mTextSettingName;
        private Button mButton;

        public ButtonSettingViewHolder(View itemView) {
            super(itemView);
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mButton = root.findViewById(R.id.button_setting);
            mButton.setOnClickListener(this);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mTextSettingName.setText(item.getName());
            mButton.setText(R.string.multiplayer_kick_member);
        }

        @Override
        public void onClick(View clicked) {
            if (mItem == null) {
                return;
            }

            if (mItem.getSetting() == SettingsItem.SETTING_MULTIPLAYER_ROOM_MEMBER) {
                NetPlayManager.NetPlayKickUser(mItem.getName());
            }
        }
    }

    public class SettingsAdapter extends RecyclerView.Adapter<SettingViewHolder> {
        private int[] mInitialRunningSettings;
        private int[] mRunningSettings;
        private int mInitialUseHapticFeedback = Integer.MIN_VALUE;
        private int mUseHapticFeedback;
        private int mInitialJoystickRelative = Integer.MIN_VALUE;
        private int mJoystickRelative;
        private int mInitialHideInputOverlay = Integer.MIN_VALUE;
        private int mHideInputOverlay;
        private int mInitialControllerScale = Integer.MIN_VALUE;
        private int mControllerScale;
        private int mInitialControllerAlpha = Integer.MIN_VALUE;
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
            ensureWorkingStateLoaded();
            mSettings = new ArrayList<>();

            // pref settings
            mSettings.add(new SettingsItem(SettingsItem.SETTING_USE_HAPTIC_FEEDBACK,
                    R.string.use_haptic_feedback,
                    SettingsItem.TYPE_CHECKBOX, mUseHapticFeedback));

            mSettings.add(new SettingsItem(SettingsItem.SETTING_JOYSTICK_RELATIVE,
                    R.string.joystick_relative_center,
                    SettingsItem.TYPE_CHECKBOX, mJoystickRelative));

            mSettings.add(new SettingsItem(SettingsItem.SETTING_HIDE_INPUT_OVERLAY,
                    R.string.hide_input_overlay,
                    SettingsItem.TYPE_CHECKBOX, mHideInputOverlay));

            mSettings.add(new SettingsItem(SettingsItem.SETTING_CONTROLLER_SCALE,
                    R.string.controller_scale, SettingsItem.TYPE_SEEK_BAR,
                    mControllerScale));

            mSettings.add(new SettingsItem(SettingsItem.SETTING_CONTROLLER_ALPHA,
                    R.string.controller_alpha, SettingsItem.TYPE_SEEK_BAR,
                    mControllerAlpha));

            // native settings
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CORE_TICKS_HACK,
                    R.string.setting_core_ticks_hack,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_CORE_TICKS_HACK]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_SLOW_DRAW,
                    R.string.setting_skip_slow_draw,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_SKIP_SLOW_DRAW]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_CPU_WRITE,
                    R.string.setting_skip_cpu_write,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_SKIP_CPU_WRITE]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_TEXTURE_COPY,
                    R.string.setting_skip_texture_copy,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_SKIP_TEXTURE_COPY]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_FORCE_TEXTURE_FILTER,
                    R.string.setting_force_texture_filter,
                    SettingsItem.TYPE_RADIO_GROUP, mRunningSettings[SettingsItem.SETTING_FORCE_TEXTURE_FILTER]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_HW_GS_MODE,
                    R.string.setting_hw_gs_mode,
                    SettingsItem.TYPE_RADIO_GROUP, mRunningSettings[SettingsItem.SETTING_HW_GS_MODE]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SHADOW_RENDERING,
                    R.string.setting_shadow_rendering,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_SHADOW_RENDERING]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_ASYNC_SHADER_COMPILE,
                    R.string.setting_async_shader_compile,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_ASYNC_SHADER_COMPILE]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_USE_COMPATIBLE_MODE,
                    R.string.setting_use_compatible_mode,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_USE_COMPATIBLE_MODE]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SCALE_FACTOR,
                    R.string.running_resolution, SettingsItem.TYPE_RADIO_GROUP,
                    mRunningSettings[SettingsItem.SETTING_SCALE_FACTOR]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LOAD_SUBMENU,
                    R.string.setting_screen_layout_settings, SettingsItem.TYPE_TEXT,
                    MENU_SCREEN_LAYOUT));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_ACCURATE_MUL,
                    R.string.running_accurate_mul, SettingsItem.TYPE_RADIO_GROUP,
                    mRunningSettings[SettingsItem.SETTING_ACCURATE_MUL]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CUSTOM_LAYOUT,
                    R.string.running_custom_layout,
                    SettingsItem.TYPE_CHECKBOX, mRunningSettings[SettingsItem.SETTING_CUSTOM_LAYOUT]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_FRAME_LIMIT,
                    R.string.running_frame_limit,
                    SettingsItem.TYPE_SEEK_BAR, mRunningSettings[SettingsItem.SETTING_FRAME_LIMIT]));
            notifyDataSetChanged();
        }

        public void loadScreenLayoutMenu() {
            ensureWorkingStateLoaded();
            mSettings = new ArrayList<>();
            final int currentLayout =
                normalizeLayoutOption(mRunningSettings[SettingsItem.SETTING_SCREEN_LAYOUT]);

            mSettings.add(new SettingsItem(SettingsItem.SETTING_SCREEN_LAYOUT,
                    R.string.running_layout, SettingsItem.TYPE_RADIO_GROUP,
                    currentLayout));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SWAP_SCREEN,
                    R.string.swap_screens, SettingsItem.TYPE_CHECKBOX,
                    mRunningSettings[SettingsItem.SETTING_SWAP_SCREEN]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_DYNAMIC_SCREEN_FILL,
                    R.string.dynamic_screen_fill, SettingsItem.TYPE_CHECKBOX,
                    mRunningSettings[SettingsItem.SETTING_DYNAMIC_SCREEN_FILL]));

            if (currentLayout == 2) {
                final boolean autoFitEnabled =
                    mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT] > 0;
                mSettings.add(new SettingsItem(SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT,
                        R.string.large_screen_auto_fit, SettingsItem.TYPE_CHECKBOX,
                        mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT]));
                if (!autoFitEnabled) {
                    mSettings.add(new SettingsItem(SettingsItem.SETTING_LARGE_SCREEN_PROPORTION,
                            R.string.running_large_screen_proportion, SettingsItem.TYPE_SEEK_BAR,
                            mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_PROPORTION]));
                }
                mSettings.add(new SettingsItem(SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_LEFT,
                        R.string.large_screen_secondary_side, SettingsItem.TYPE_RADIO_GROUP,
                        mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_LEFT]));
                mSettings.add(new SettingsItem(SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_TOP,
                        R.string.large_screen_secondary_vertical_alignment,
                        SettingsItem.TYPE_RADIO_GROUP,
                        mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_TOP]));
            } else if (currentLayout == 5) {
                mSettings.add(new SettingsItem(SettingsItem.SETTING_HYBRID_SIDE_COLUMN_LEFT,
                        R.string.hybrid_side_column_side, SettingsItem.TYPE_RADIO_GROUP,
                        mRunningSettings[SettingsItem.SETTING_HYBRID_SIDE_COLUMN_LEFT]));
                mSettings.add(new SettingsItem(SettingsItem.SETTING_HYBRID_SECONDARY_TOP,
                        R.string.hybrid_secondary_vertical_order, SettingsItem.TYPE_RADIO_GROUP,
                        mRunningSettings[SettingsItem.SETTING_HYBRID_SECONDARY_TOP]));
            }

            mSettings.add(new SettingsItem(SettingsItem.SETTING_LAYOUT_MARGIN_LEFT,
                    R.string.layout_margin_left, SettingsItem.TYPE_SEEK_BAR,
                    mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_LEFT]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LAYOUT_MARGIN_TOP,
                    R.string.layout_margin_top, SettingsItem.TYPE_SEEK_BAR,
                    mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_TOP]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT,
                    R.string.layout_margin_right, SettingsItem.TYPE_SEEK_BAR,
                    mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM,
                    R.string.layout_margin_bottom, SettingsItem.TYPE_SEEK_BAR,
                    mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM]));
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

            if (mInitialUseHapticFeedback != mUseHapticFeedback) {
                editor.putBoolean(InputOverlay.PREF_HAPTIC_FEEDBACK, mUseHapticFeedback > 0);
                InputOverlay.sUseHapticFeedback = mUseHapticFeedback > 0;
            }
            if (mInitialJoystickRelative != mJoystickRelative) {
                editor.putBoolean(InputOverlay.PREF_JOYSTICK_RELATIVE, mJoystickRelative > 0);
                InputOverlay.sJoystickRelative = mJoystickRelative > 0;
            }
            if (mInitialHideInputOverlay != mHideInputOverlay) {
                editor.putBoolean(InputOverlay.PREF_CONTROLLER_HIDE, mHideInputOverlay > 0);
                InputOverlay.sHideInputOverlay = mHideInputOverlay > 0;
            }
            if (mInitialControllerScale != mControllerScale) {
                editor.putInt(InputOverlay.PREF_CONTROLLER_SCALE, mControllerScale);
                InputOverlay.sControllerScale = mControllerScale;
            }
            if (mInitialControllerAlpha != mControllerAlpha) {
                editor.putInt(InputOverlay.PREF_CONTROLLER_ALPHA, mControllerAlpha);
                InputOverlay.sControllerAlpha = mControllerAlpha;
            }

            // applay prefs
            editor.apply();
            activity.refreshControls();

            // native settings
            boolean isChanged = false;
            for (int i = 0; i < mRunningSettings.length; ++i) {
                if (mInitialRunningSettings[i] != mRunningSettings[i]) {
                    isChanged = true;
                    break;
                }
            }
            if (isChanged) {
                NativeLibrary.setRunningSettings(mRunningSettings);
            }
            mInitialRunningSettings = mRunningSettings.clone();
            mInitialUseHapticFeedback = mUseHapticFeedback;
            mInitialJoystickRelative = mJoystickRelative;
            mInitialHideInputOverlay = mHideInputOverlay;
            mInitialControllerScale = mControllerScale;
            mInitialControllerAlpha = mControllerAlpha;
        }

        public void updateWorkingValue(int setting, int value) {
            if (setting >= 0 && mRunningSettings != null && setting < mRunningSettings.length) {
                mRunningSettings[setting] = value;
                if (shouldApplyLargeScreenTopAutoFit(setting)) {
                    mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_PROPORTION] =
                        Math.max(25, Math.min(100, getLargeScreenTopAutoFitForCurrentDisplay()));
                }
                if (isLiveScreenLayoutSetting(setting)) {
                    NativeLibrary.setRunningSettings(mRunningSettings);
                }
                if (shouldReloadScreenLayoutMenu(setting)) {
                    loadScreenLayoutMenu();
                }
                return;
            }

            switch (setting) {
            case SettingsItem.SETTING_USE_HAPTIC_FEEDBACK:
                mUseHapticFeedback = value;
                break;
            case SettingsItem.SETTING_JOYSTICK_RELATIVE:
                mJoystickRelative = value;
                break;
            case SettingsItem.SETTING_HIDE_INPUT_OVERLAY:
                mHideInputOverlay = value;
                break;
            case SettingsItem.SETTING_CONTROLLER_SCALE:
                mControllerScale = value;
                break;
            case SettingsItem.SETTING_CONTROLLER_ALPHA:
                mControllerAlpha = value;
                break;
            default:
                break;
            }
        }

        private boolean isLiveScreenLayoutSetting(int setting) {
            return setting == SettingsItem.SETTING_SCREEN_LAYOUT ||
                   setting == SettingsItem.SETTING_SWAP_SCREEN ||
                   setting == SettingsItem.SETTING_DYNAMIC_SCREEN_FILL ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_PROPORTION ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_LEFT ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_TOP ||
                   setting == SettingsItem.SETTING_HYBRID_SIDE_COLUMN_LEFT ||
                   setting == SettingsItem.SETTING_HYBRID_SECONDARY_TOP ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_LEFT ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_TOP ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM;
        }

        private boolean shouldApplyLargeScreenTopAutoFit(int setting) {
            if (mRunningSettings == null ||
                normalizeLayoutOption(mRunningSettings[SettingsItem.SETTING_SCREEN_LAYOUT]) != 2) {
                return false;
            }

            if (setting == SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT) {
                return mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT] > 0;
            }

            if (mRunningSettings[SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT] <= 0) {
                return false;
            }

            return setting == SettingsItem.SETTING_SWAP_SCREEN ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_LEFT ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_SECONDARY_TOP ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_LEFT ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_TOP ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT ||
                   setting == SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM;
        }

        private boolean shouldReloadScreenLayoutMenu(int setting) {
            return setting == SettingsItem.SETTING_SCREEN_LAYOUT ||
                   setting == SettingsItem.SETTING_LARGE_SCREEN_AUTO_FIT;
        }

        private int normalizeLayoutOption(int layoutOption) {
            return layoutOption == 4 ? 2 : layoutOption;
        }

        public int getLargeScreenTopAutoFitForCurrentDisplay() {
            final EmulationActivity activity =
                (EmulationActivity)NativeLibrary.getEmulationContext();
            if (activity == null || mRunningSettings == null) {
                return NativeLibrary.getLargeScreenTopAutoFitProportion();
            }

            final DisplayMetrics metrics = new DisplayMetrics();
            activity.getWindowManager().getDefaultDisplay().getRealMetrics(metrics);
            return NativeLibrary.getLargeScreenTopAutoFitProportionForDimensions(
                metrics.widthPixels, metrics.heightPixels,
                mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_LEFT],
                mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_TOP],
                mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_RIGHT],
                mRunningSettings[SettingsItem.SETTING_LAYOUT_MARGIN_BOTTOM],
                mRunningSettings[SettingsItem.SETTING_SWAP_SCREEN] > 0);
        }

        private void ensureWorkingStateLoaded() {
            if (mInitialRunningSettings == null) {
                mInitialRunningSettings = NativeLibrary.getRunningSettings();
                mRunningSettings = mInitialRunningSettings.clone();
            }

            if (mInitialUseHapticFeedback == Integer.MIN_VALUE) {
                mInitialUseHapticFeedback = InputOverlay.sUseHapticFeedback ? 1 : 0;
                mUseHapticFeedback = mInitialUseHapticFeedback;
                mInitialJoystickRelative = InputOverlay.sJoystickRelative ? 1 : 0;
                mJoystickRelative = mInitialJoystickRelative;
                mInitialHideInputOverlay = InputOverlay.sHideInputOverlay ? 1 : 0;
                mHideInputOverlay = mInitialHideInputOverlay;
                mInitialControllerScale = InputOverlay.sControllerScale;
                mControllerScale = mInitialControllerScale;
                mInitialControllerAlpha = InputOverlay.sControllerAlpha;
                mControllerAlpha = mInitialControllerAlpha;
            }
        }
    }
}
