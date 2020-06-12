package org.citra.emu.settings.viewholder;

import android.view.View;
import android.widget.SeekBar;
import android.widget.TextView;
import org.citra.emu.R;
import org.citra.emu.settings.SettingsAdapter;
import org.citra.emu.settings.view.SettingsItem;
import org.citra.emu.settings.view.SliderSetting;

public final class SeekbarViewHolder extends SettingViewHolder {
    private SliderSetting mItem;

    private TextView mName;
    private TextView mValue;
    private SeekBar mSeekBar;

    public SeekbarViewHolder(View itemView, SettingsAdapter adapter) {
        super(itemView, adapter);
    }

    @Override
    protected void findViews(View root) {
        mName = root.findViewById(R.id.text_setting_name);
        mValue = root.findViewById(R.id.text_setting_value);
        mSeekBar = root.findViewById(R.id.seekbar);
    }

    @Override
    public void bind(SettingsItem item) {
        mItem = (SliderSetting)item;
        mName.setText(item.getNameId());
        mSeekBar.setMax(mItem.getMax());
        mSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean b) {
                if (mItem.getMax() > 99)
                    progress = (progress / 5) * 5;
                mValue.setText(progress + mItem.getUnits());
                if (progress != mItem.getSelectedValue()) {
                    mItem.setSelectedValue(progress);
                    getAdapter().onSeekbarClick(mItem, getAdapterPosition(), progress);
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
        mSeekBar.setProgress(mItem.getSelectedValue());
    }

    @Override
    public void onClick(View clicked) {}
}
