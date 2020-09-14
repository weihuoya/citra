package org.citra.emu.settings;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.text.Html;
import android.text.Spanned;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.recyclerview.widget.RecyclerView;
import androidx.preference.PreferenceManager;

import java.util.ArrayList;
import org.citra.emu.R;
import org.citra.emu.settings.model.BooleanSetting;
import org.citra.emu.settings.model.IntSetting;
import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.StringSetting;
import org.citra.emu.settings.view.CheckBoxSetting;
import org.citra.emu.settings.view.EditorSetting;
import org.citra.emu.settings.view.InputBindingSetting;
import org.citra.emu.settings.view.SettingsItem;
import org.citra.emu.settings.view.SingleChoiceSetting;
import org.citra.emu.settings.view.SliderSetting;
import org.citra.emu.settings.view.StringSingleChoiceSetting;
import org.citra.emu.settings.view.SubmenuSetting;
import org.citra.emu.settings.viewholder.CheckBoxSettingViewHolder;
import org.citra.emu.settings.viewholder.EditorViewHolder;
import org.citra.emu.settings.viewholder.HeaderViewHolder;
import org.citra.emu.settings.viewholder.InputBindingSettingViewHolder;
import org.citra.emu.settings.viewholder.SeekbarViewHolder;
import org.citra.emu.settings.viewholder.SettingViewHolder;
import org.citra.emu.settings.viewholder.SingleChoiceViewHolder;
import org.citra.emu.settings.viewholder.SliderViewHolder;
import org.citra.emu.settings.viewholder.SubmenuViewHolder;
import org.citra.emu.utils.PermissionsHandler;

public final class SettingsAdapter extends RecyclerView.Adapter<SettingViewHolder>
    implements DialogInterface.OnClickListener, SeekBar.OnSeekBarChangeListener {
    private SettingsActivity mActivity;
    private ArrayList<SettingsItem> mSettings;

    private SettingsItem mClickedItem;
    private int mClickedPosition;

    private AlertDialog mDialog;
    private TextView mTextSliderValue;

    public SettingsAdapter(SettingsActivity activity) {
        mActivity = activity;
        mClickedPosition = -1;
    }

    @Override
    public SettingViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View view;
        LayoutInflater inflater = LayoutInflater.from(parent.getContext());

        switch (viewType) {
        case SettingsItem.TYPE_HEADER:
            view = inflater.inflate(R.layout.list_item_settings_header, parent, false);
            return new HeaderViewHolder(view, this);

        case SettingsItem.TYPE_CHECKBOX:
            view = inflater.inflate(R.layout.list_item_setting_checkbox, parent, false);
            return new CheckBoxSettingViewHolder(view, this);

        case SettingsItem.TYPE_STRING_SINGLE_CHOICE:
        case SettingsItem.TYPE_SINGLE_CHOICE:
            view = inflater.inflate(R.layout.list_item_setting, parent, false);
            return new SingleChoiceViewHolder(view, this);

        case SettingsItem.TYPE_SLIDER:
            view = inflater.inflate(R.layout.list_item_setting, parent, false);
            return new SliderViewHolder(view, this);

        case SettingsItem.TYPE_SUBMENU:
            view = inflater.inflate(R.layout.list_item_setting, parent, false);
            return new SubmenuViewHolder(view, this);

        case SettingsItem.TYPE_INPUT_BINDING:
            view = inflater.inflate(R.layout.list_item_setting, parent, false);
            return new InputBindingSettingViewHolder(view, this);

        case SettingsItem.TYPE_SEEKBAR:
            view = inflater.inflate(R.layout.list_item_setting_seekbar, parent, false);
            return new SeekbarViewHolder(view, this);

        case SettingsItem.TYPE_EDITOR:
            view = inflater.inflate(R.layout.list_item_setting, parent, false);
            return new EditorViewHolder(view, this);

        default:
            Log.e("citra", "[SettingsAdapter] Invalid view type: " + viewType);
            return null;
        }
    }

    @Override
    public void onBindViewHolder(SettingViewHolder holder, int position) {
        holder.bind(mSettings.get(position));
    }

    @Override
    public int getItemCount() {
        if (mSettings != null) {
            return mSettings.size();
        } else {
            return 0;
        }
    }

    @Override
    public int getItemViewType(int position) {
        return mSettings.get(position).getType();
    }

    public String getSettingSection(int position) {
        return mSettings.get(position).getSection();
    }

    public void setSettings(ArrayList<SettingsItem> settings) {
        mSettings = settings;
        notifyDataSetChanged();
    }

    public void onBooleanClick(CheckBoxSetting item, int position, boolean checked) {
        BooleanSetting setting = item.setChecked(checked);
        if (setting != null) {
            mActivity.putSetting(setting);
        }
        mActivity.setSettingChanged();
    }

    public void onSingleChoiceClick(SingleChoiceSetting item, int position) {
        mClickedItem = item;
        mClickedPosition = position;

        int value = getSelectionForSingleChoiceValue(item);
        AlertDialog.Builder builder = new AlertDialog.Builder(mActivity);
        builder.setTitle(item.getNameId());
        builder.setSingleChoiceItems(item.getChoicesId(), value, this);
        mDialog = builder.show();
    }

    public void onStringSingleChoiceClick(StringSingleChoiceSetting item, int position) {
        mClickedItem = item;
        mClickedPosition = position;

        AlertDialog.Builder builder = new AlertDialog.Builder(mActivity);
        builder.setTitle(item.getNameId());
        builder.setSingleChoiceItems(item.getChoicesId(), item.getSelectValueIndex(), this);
        mDialog = builder.show();
    }

    public void onSeekbarClick(SliderSetting item, int position, int progress) {
        Setting setting = item.setSelectedValue(progress);
        if (setting != null) {
            mActivity.putSetting(setting);
        }
        mActivity.setSettingChanged();
    }

    public void onSliderClick(SliderSetting item, int position) {
        mClickedItem = item;
        mClickedPosition = position;
        int progress = item.getSelectedValue();
        AlertDialog.Builder builder = new AlertDialog.Builder(mActivity);

        LayoutInflater inflater = LayoutInflater.from(mActivity);
        View view = inflater.inflate(R.layout.dialog_seekbar, null);

        builder.setTitle(item.getNameId());
        builder.setView(view);
        builder.setPositiveButton(android.R.string.ok, this);
        mDialog = builder.show();

        mTextSliderValue = view.findViewById(R.id.text_value);
        mTextSliderValue.setText(String.valueOf(progress));

        TextView units = view.findViewById(R.id.text_units);
        units.setText(item.getUnits());

        SeekBar seekbar = view.findViewById(R.id.seekbar);
        seekbar.setMax(item.getMax());
        seekbar.setProgress(progress);
        seekbar.setKeyProgressIncrement(5);
        seekbar.setOnSeekBarChangeListener(this);
    }

    public void onSubmenuClick(SubmenuSetting item) {
        mActivity.loadSubMenu(item.getMenuKey());
    }

    public void onEditorClick(EditorSetting item, int position) {
        mClickedItem = item;
        mClickedPosition = position;
        AlertDialog.Builder builder = new AlertDialog.Builder(mActivity);

        LayoutInflater inflater = LayoutInflater.from(mActivity);
        View view = inflater.inflate(R.layout.dialog_editor, null);

        builder.setTitle(item.getNameId());
        builder.setView(view);
        builder.setPositiveButton(android.R.string.ok, this);
        mDialog = builder.show();

        EditText editor = view.findViewById(R.id.setting_editor);
        editor.setText(item.getSelectedValue());
        editor.requestFocus();
    }

    private Spanned getFormatString(int resId, String arg) {
        String unspanned = String.format(mActivity.getString(resId), arg);
        Spanned spanned;
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.N) {
            spanned = Html.fromHtml(unspanned, Html.FROM_HTML_MODE_LEGACY);
        } else {
            spanned = Html.fromHtml(unspanned);
        }
        return spanned;
    }

    public void onInputBindingClick(final InputBindingSetting item, final int position) {
        mClickedItem = item;
        mClickedPosition = position;

        final MotionAlertDialog dialog = new MotionAlertDialog(mActivity, item);
        dialog.setTitle(R.string.input_binding);
        dialog.setMessage(getFormatString(R.string.input_binding_description,
                                          mActivity.getString(item.getNameId())));
        dialog.setButton(AlertDialog.BUTTON_NEGATIVE, mActivity.getString(android.R.string.cancel),
                         this);
        dialog.setButton(AlertDialog.BUTTON_NEUTRAL,
                         mActivity.getString(R.string.clear_input_binding),
                         (dialogInterface, i) -> {
                             SharedPreferences preferences =
                                 PreferenceManager.getDefaultSharedPreferences(mActivity);
                             item.clearValue();
                         });
        dialog.setOnDismissListener(dialog1 -> {
            StringSetting setting =
                new StringSetting(item.getKey(), item.getSection(), item.getValue());
            notifyItemChanged(position);
            mActivity.putSetting(setting);
            mActivity.setSettingChanged();
        });
        dialog.setCanceledOnTouchOutside(false);
        dialog.show();
    }

    @Override
    public void onClick(DialogInterface dialog, int which) {
        if (mClickedItem instanceof SingleChoiceSetting) {
            SingleChoiceSetting scSetting = (SingleChoiceSetting)mClickedItem;

            int value = getValueForSingleChoiceSelection(scSetting, which);
            if (scSetting.getSelectedValue() != value)
                mActivity.setSettingChanged();

            // Get the backing Setting, which may be null (if for example it was missing from the
            // file)
            IntSetting setting = scSetting.setSelectedValue(value);
            if (scSetting.getSetting().getKey().equals(SettingsFile.KEY_MIC_INPUT_TYPE) &&
                value == 1) {
                PermissionsHandler.checkRecordPermission(mActivity);
            }
            if (setting != null) {
                mActivity.putSetting(setting);
            } else {
                //
            }

            closeDialog();
        } else if (mClickedItem instanceof StringSingleChoiceSetting) {
            StringSingleChoiceSetting scSetting = (StringSingleChoiceSetting)mClickedItem;
            String value = scSetting.getValueAt(which);
            if (!scSetting.getSelectedValue().equals(value))
                mActivity.setSettingChanged();

            StringSetting setting = scSetting.setSelectedValue(value);
            if (setting != null) {
                mActivity.putSetting(setting);
            }

            closeDialog();
        } else if (mClickedItem instanceof SliderSetting) {
            SliderSetting sliderSetting = (SliderSetting)mClickedItem;
            SeekBar seekBar = mDialog.findViewById(R.id.seekbar);
            int value = seekBar.getProgress();
            if (sliderSetting.getSelectedValue() != value)
                mActivity.setSettingChanged();

            Setting setting = sliderSetting.setSelectedValue(value);
            if (setting != null) {
                mActivity.putSetting(setting);
            }

            closeDialog();
        } else if (mClickedItem instanceof EditorSetting) {
            EditorSetting editorSetting = (EditorSetting)mClickedItem;
            EditText editor = mDialog.findViewById(R.id.setting_editor);
            String value = editor.getText().toString();
            if (!editorSetting.getSelectedValue().equals(value))
                mActivity.setSettingChanged();

            Setting setting = editorSetting.setSelectedValue(value);
            if (setting != null) {
                mActivity.putSetting(setting);
            }

            closeDialog();
        }

        mClickedItem = null;
    }

    public void closeDialog() {
        if (mDialog != null) {
            if (mClickedPosition != -1) {
                notifyItemChanged(mClickedPosition);
                mClickedPosition = -1;
            }
            mDialog.dismiss();
            mDialog = null;
        }
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        progress = seekBar.getMax() > 99 ? (progress / 5) * 5 : progress;
        mTextSliderValue.setText(String.valueOf(progress));
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {}

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {}

    private int getValueForSingleChoiceSelection(SingleChoiceSetting item, int which) {
        int valuesId = item.getValuesId();

        if (valuesId > 0) {
            int[] valuesArray = mActivity.getResources().getIntArray(valuesId);
            return valuesArray[which];
        } else {
            return which;
        }
    }

    private int getSelectionForSingleChoiceValue(SingleChoiceSetting item) {
        int value = item.getSelectedValue();
        int valuesId = item.getValuesId();

        if (valuesId > 0) {
            int[] valuesArray = mActivity.getResources().getIntArray(valuesId);
            for (int index = 0; index < valuesArray.length; index++) {
                int current = valuesArray[index];
                if (current == value) {
                    return index;
                }
            }
        } else {
            return value;
        }

        return -1;
    }
}
