package org.citra.emu.settings.viewholder;

import android.view.View;
import android.widget.TextView;
import org.citra.emu.R;
import org.citra.emu.settings.SettingsAdapter;
import org.citra.emu.settings.view.SettingsItem;
import org.citra.emu.settings.view.SubmenuSetting;

public final class SubmenuViewHolder extends SettingViewHolder {
    private SubmenuSetting mItem;

    private TextView mTextSettingName;
    private TextView mTextSettingDescription;

    public SubmenuViewHolder(View itemView, SettingsAdapter adapter) {
        super(itemView, adapter);
    }

    @Override
    protected void findViews(View root) {
        mTextSettingName = root.findViewById(R.id.text_setting_name);
        mTextSettingDescription = root.findViewById(R.id.text_setting_description);
    }

    @Override
    public void bind(SettingsItem item) {
        mItem = (SubmenuSetting)item;

        mTextSettingName.setText(item.getNameId());

        if (item.getDescriptionId() > 0) {
            mTextSettingDescription.setText(item.getDescriptionId());
        }
    }

    @Override
    public void onClick(View clicked) {
        getAdapter().onSubmenuClick(mItem);
    }
}
