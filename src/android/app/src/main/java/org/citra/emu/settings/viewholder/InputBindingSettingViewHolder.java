package org.citra.emu.settings.viewholder;

import android.view.View;
import android.widget.TextView;

import org.citra.emu.R;
import org.citra.emu.settings.SettingsAdapter;
import org.citra.emu.settings.view.InputBindingSetting;
import org.citra.emu.settings.view.SettingsItem;

public final class InputBindingSettingViewHolder extends SettingViewHolder {
    private InputBindingSetting mItem;

    private TextView mTextSettingName;
    private TextView mTextSettingDescription;

    public InputBindingSettingViewHolder(View itemView, SettingsAdapter adapter) {
        super(itemView, adapter);
    }

    @Override
    protected void findViews(View root) {
        mTextSettingName = root.findViewById(R.id.text_setting_name);
        mTextSettingDescription = root.findViewById(R.id.text_setting_description);
    }

    @Override
    public void bind(SettingsItem item) {
        mItem = (InputBindingSetting) item;
        mTextSettingName.setText(mItem.getNameId());
        mTextSettingDescription.setText(mItem.getSettingText());
    }

    @Override
    public void onClick(View clicked) {
        getAdapter().onInputBindingClick(mItem, getAdapterPosition());
    }
}
