package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.annotation.NonNull;
import android.support.v4.app.DialogFragment;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import com.nononsenseapps.filepicker.DividerItemDecoration;
import java.util.ArrayList;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.overlay.InputOverlay;

public class RunningSettingDialog extends DialogFragment {
    private SettingsAdapter mAdapter;

    public static RunningSettingDialog newInstance() {
        return new RunningSettingDialog();
    }

    @NonNull
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        ViewGroup contents = (ViewGroup)getActivity().getLayoutInflater().inflate(
            R.layout.dialog_running_settings, null);

        Drawable lineDivider = getContext().getDrawable(R.drawable.line_divider);
        RecyclerView recyclerView = contents.findViewById(R.id.list_settings);
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(getContext());
        recyclerView.setLayoutManager(layoutManager);
        mAdapter = new SettingsAdapter();
        recyclerView.setAdapter(mAdapter);
        recyclerView.addItemDecoration(new DividerItemDecoration(lineDivider));
        builder.setView(contents);
        return builder.create();
    }

    @Override
    public void onDismiss(DialogInterface dialog) {
        super.onDismiss(dialog);
        mAdapter.saveSettings();
    }

    public class SettingsItem {
        // setting type
        public static final int SETTING_CORE_TICKS_HACK = 0;
        public static final int SETTING_SKIP_SLOW_DRAW = 1;
        public static final int SETTING_SCALE_FACTOR = 2;
        public static final int SETTING_SCREEN_LAYOUT = 3;
        public static final int SETTING_TEXTURE_LOAD_HACK = 4;

        // pref
        public static final int SETTING_JOYSTICK_RELATIVE = 100;
        public static final int SETTING_HIDE_INPUT_OVERLAY = 101;
        public static final int SETTING_CONTROLLER_SCALE = 102;
        public static final int SETTING_CONTROLLER_ALPHA = 103;
        public static final int SETTING_EMULATE_MOTION_BY_TOUCH = 104;

        // view type
        public static final int TYPE_CHECKBOX = 0;
        public static final int TYPE_RADIO_GROUP = 1;
        public static final int TYPE_SEEK_BAR = 2;

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

        @Override protected void findViews(View root)
        {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mRadioGroup = root.findViewById(R.id.radio_group);
            mRadioGroup.setOnCheckedChangeListener(this);
        }

        @Override public void bind(SettingsItem item)
        {
            int checkIds[] = {R.id.radio0, R.id.radio1, R.id.radio2, R.id.radio3};
            int index = item.getValue();
            if (index < 0 || index >= checkIds.length)
                index = 0;

            mItem = item;
            mTextSettingName.setText(item.getName());
            mRadioGroup.check(checkIds[index]);

            if (item.getSetting() == SettingsItem.SETTING_SCREEN_LAYOUT)
            {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText(R.string.default_value);

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText(R.string.single_screen_option);

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText(R.string.large_screen_option);

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setText(R.string.side_screen_option);
            }
            else if (item.getSetting() == SettingsItem.SETTING_SCALE_FACTOR)
            {
                RadioButton radio0 = mRadioGroup.findViewById(R.id.radio0);
                radio0.setText("×1");

                RadioButton radio1 = mRadioGroup.findViewById(R.id.radio1);
                radio1.setText("×2");

                RadioButton radio2 = mRadioGroup.findViewById(R.id.radio2);
                radio2.setText("×3");

                RadioButton radio3 = mRadioGroup.findViewById(R.id.radio3);
                radio3.setText("×4");
            }
        }

        @Override public void onClick(View clicked) {}

        @Override public void onCheckedChanged(RadioGroup group, int checkedId)
        {
            switch (checkedId)
            {
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

        public SeekBarSettingViewHolder(View itemView) {
            super(itemView);
        }

        @Override
        protected void findViews(View root) {
            mTextSettingName = root.findViewById(R.id.text_setting_name);
            mTextSettingValue = root.findViewById(R.id.text_setting_value);
            mSeekBar = root.findViewById(R.id.seekbar);
            mSeekBar.setProgress(99);
        }

        @Override
        public void bind(SettingsItem item) {
            mItem = item;
            mTextSettingName.setText(item.getName());
            mSeekBar.setMax(100);
            mSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean b) {
                    if (seekBar.getMax() > 99) {
                        progress = (progress / 5) * 5;
                        mTextSettingValue.setText(progress + "%");
                    } else {
                        mTextSettingValue.setText(String.valueOf(progress));
                    }
                    mItem.setValue(progress);
                }

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {}
            });
            mSeekBar.setProgress(item.getValue());
        }

        @Override
        public void onClick(View clicked) {}
    }

    public class SettingsAdapter extends RecyclerView.Adapter<SettingViewHolder> {
        private int[] mRunningSettings;
        private int mJoystickRelative;
        private int mHideInputOverlay;
        private int mControllerScale;
        private int mControllerAlpha;
        private int mEmulateMotionByTouch;
        private ArrayList<SettingsItem> mSettings;

        public SettingsAdapter() {
            int i = 0;
            EmulationActivity activity = (EmulationActivity)NativeLibrary.getEmulationContext();
            mRunningSettings = NativeLibrary.getRunningSettings();
            mSettings = new ArrayList<>();

            // pref settings
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

            mEmulateMotionByTouch = InputOverlay.sEmulateMotionByTouch ? 1 : 0;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_EMULATE_MOTION_BY_TOUCH,
                                           R.string.emulate_motion_by_touch,
                                           SettingsItem.TYPE_CHECKBOX, mEmulateMotionByTouch));

            // native settings
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CORE_TICKS_HACK,
                                           R.string.setting_core_ticks_hack,
                                           SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SKIP_SLOW_DRAW,
                                           R.string.setting_skip_slow_draw,
                                           SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_TEXTURE_LOAD_HACK,
                                           R.string.setting_texture_load_hack,
                                           SettingsItem.TYPE_CHECKBOX, mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SCALE_FACTOR,
                                           R.string.running_resolution, SettingsItem.TYPE_RADIO_GROUP,
                                           mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_SCREEN_LAYOUT,
                                           R.string.running_layout, SettingsItem.TYPE_RADIO_GROUP,
                                           mRunningSettings[i++]));
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
            EmulationActivity activity = (EmulationActivity)NativeLibrary.getEmulationContext();

            // pref settings
            SharedPreferences.Editor editor =
                PreferenceManager.getDefaultSharedPreferences(activity).edit();
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

            int motion = mSettings.get(0).getValue();
            if (mEmulateMotionByTouch != motion) {
                InputOverlay.sEmulateMotionByTouch = motion > 0;
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
        }
    }
}
