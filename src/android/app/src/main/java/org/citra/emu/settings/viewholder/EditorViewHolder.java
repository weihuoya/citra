package org.citra.emu.settings.viewholder;

import android.view.View;
import android.widget.TextView;
import org.citra.emu.R;
import org.citra.emu.settings.SettingsAdapter;
import org.citra.emu.settings.view.EditorSetting;
import org.citra.emu.settings.view.SettingsItem;

public final class EditorViewHolder extends SettingViewHolder {
    private EditorSetting mItem;

    private TextView mTextSettingName;
    private TextView mTextSettingDescription;

    public EditorViewHolder(View itemView, SettingsAdapter adapter) {
        super(itemView, adapter);
    }

    @Override
    protected void findViews(View root) {
        mTextSettingName = root.findViewById(R.id.text_setting_name);
        mTextSettingDescription = root.findViewById(R.id.text_setting_description);
    }

    @Override
    public void bind(SettingsItem item) {
        mItem = (EditorSetting)item;
        mTextSettingName.setText(item.getNameId());
        mTextSettingDescription.setText(mItem.getSelectedValue());
    }

    @Override
    public void onClick(View clicked) {
        getAdapter().onEditorClick(mItem, getAdapterPosition());
    }
}
