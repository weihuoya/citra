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

        int columns = 1;
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
        public static final int SETTING_LAYOUT_SINGLE_SCREEN = 1;

        // pref
        public static final int SETTING_JOYSTICK_RELATIVE = 100;

        // view type
        public static final int TYPE_CHECKBOX = 0;
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
        private ArrayList<SettingsItem> mSettings;

        public SettingsAdapter() {
            int i = 0;
            SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getContext());
            mRunningSettings = NativeLibrary.getRunningSettings();
            mSettings = new ArrayList<>();

            // pref settings
            mJoystickRelative = prefs.getBoolean(InputOverlay.RELATIVE_PREF_KEY, true) ? 1 : 0;
            mSettings.add(new SettingsItem(SettingsItem.SETTING_JOYSTICK_RELATIVE,
                    R.string.joystick_relative_center, SettingsItem.TYPE_CHECKBOX,
                    mJoystickRelative));

            // native settings
            mSettings.add(new SettingsItem(SettingsItem.SETTING_CORE_TICKS_HACK,
                    R.string.setting_core_ticks_hack, SettingsItem.TYPE_SEEK_BAR,
                    mRunningSettings[i++]));
            mSettings.add(new SettingsItem(SettingsItem.SETTING_LAYOUT_SINGLE_SCREEN,
                    R.string.single_screen, SettingsItem.TYPE_CHECKBOX,
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
            // pref settings
            SharedPreferences.Editor editor = PreferenceManager.getDefaultSharedPreferences(getContext()).edit();
            int relative = mSettings.get(0).getValue();
            if (mJoystickRelative != relative) {
                editor.putBoolean(InputOverlay.RELATIVE_PREF_KEY, relative > 0);
                InputOverlay.sJoystickRelative = relative > 0;
            }
            mSettings.remove(0);
            editor.apply();

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
