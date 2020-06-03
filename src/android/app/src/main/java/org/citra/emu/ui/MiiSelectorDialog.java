package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.DialogFragment;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.TextView;
import com.nononsenseapps.filepicker.DividerItemDecoration;
import java.util.ArrayList;
import org.citra.emu.R;

public class MiiSelectorDialog extends DialogFragment {
    private static final String ARG_CANCEL = "cancel";
    private static final String ARG_TITLE = "title";
    private static final String ARG_MIIS = "miis";

    public static MiiSelectorDialog newInstance(boolean cancel, String title, String[] miis) {
        MiiSelectorDialog fragment = new MiiSelectorDialog();
        Bundle arguments = new Bundle();
        arguments.putBoolean(ARG_CANCEL, cancel);
        arguments.putString(ARG_TITLE, title);
        arguments.putStringArray(ARG_MIIS, miis);
        fragment.setArguments(arguments);
        return fragment;
    }

    @NonNull
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        ViewGroup contents = (ViewGroup)getActivity().getLayoutInflater().inflate(
            R.layout.dialog_mii_selector, null);

        Drawable lineDivider = getContext().getDrawable(R.drawable.line_divider);
        RecyclerView recyclerView = contents.findViewById(R.id.list_settings);
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(getContext());
        recyclerView.setLayoutManager(layoutManager);
        MiisAdapter adapter = new MiisAdapter();
        recyclerView.setAdapter(adapter);
        recyclerView.addItemDecoration(new DividerItemDecoration(lineDivider));
        builder.setView(contents);
        return builder.create();
    }

    @Override
    public void onDismiss(DialogInterface dialog) {
        super.onDismiss(dialog);
    }

    public class SettingsItem {}

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
            // mTextSettingName.setText(item.getName());
            // mCheckbox.setChecked(mItem.getValue() > 0);
        }

        @Override
        public void onClick(View clicked) {
            mCheckbox.toggle();
            // mItem.setValue(mCheckbox.isChecked() ? 1 : 0);
        }

        @Override
        public void onCheckedChanged(CompoundButton view, boolean isChecked) {
            // mItem.setValue(isChecked ? 1 : 0);
        }
    }

    public class MiisAdapter extends RecyclerView.Adapter<SettingViewHolder> {
        private ArrayList<SettingsItem> mSettings;

        public MiisAdapter() {}

        @NonNull
        @Override
        public SettingViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            LayoutInflater inflater = LayoutInflater.from(parent.getContext());
            View itemView = inflater.inflate(R.layout.list_item_running_checkbox, parent, false);
            return new CheckBoxSettingViewHolder(itemView);
        }

        @Override
        public int getItemCount() {
            return mSettings.size();
        }

        @Override
        public void onBindViewHolder(@NonNull SettingViewHolder holder, int position) {
            holder.bind(mSettings.get(position));
        }
    }
}
