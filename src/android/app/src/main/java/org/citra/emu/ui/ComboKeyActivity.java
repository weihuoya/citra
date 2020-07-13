// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;


import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.GridLayoutManager;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.TextView;

import com.nononsenseapps.filepicker.DividerItemDecoration;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

import java.util.ArrayList;

public final class ComboKeyActivity extends AppCompatActivity {

    private static final int[] KeyTexts = {
            R.string.button_a, R.string.button_b, R.string.button_x, R.string.button_y,
            R.string.button_r, R.string.button_l, R.string.button_zr, R.string.button_zl,
            R.string.button_up,R.string.button_down,R.string.button_left,R.string.button_right
    };
    private static final int[] KeyValues = {
            NativeLibrary.ButtonType.N3DS_BUTTON_A,
            NativeLibrary.ButtonType.N3DS_BUTTON_B,
            NativeLibrary.ButtonType.N3DS_BUTTON_X,
            NativeLibrary.ButtonType.N3DS_BUTTON_Y,
            NativeLibrary.ButtonType.N3DS_BUTTON_R,
            NativeLibrary.ButtonType.N3DS_BUTTON_L,
            NativeLibrary.ButtonType.N3DS_BUTTON_ZR,
            NativeLibrary.ButtonType.N3DS_BUTTON_ZL,
            NativeLibrary.ButtonType.N3DS_DPAD_UP,
            NativeLibrary.ButtonType.N3DS_DPAD_DOWN,
            NativeLibrary.ButtonType.N3DS_DPAD_LEFT,
            NativeLibrary.ButtonType.N3DS_DPAD_RIGHT,
    };

    private SettingAdapter mAdapter;
    private RecyclerView mListView;
    private SettingModel[] mComboKeys = new SettingModel[2];

    public static void launch(Context context) {
        Intent settings = new Intent(context, ComboKeyActivity.class);
        context.startActivity(settings);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_combo_key);

        mAdapter = new SettingAdapter();
        mListView = findViewById(R.id.list_settings);
        mListView.setAdapter(mAdapter);
        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        mListView.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(this);
        mListView.setLayoutManager(layoutManager);

        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
        for (int i = 0; i < mComboKeys.length; ++i) {
            String value = prefs.getString("combo_key_" + i, "");
            String[] keyStrs = value.split(",");
            ArrayList<Integer> keys = new ArrayList<>();
            for (String key: keyStrs) {
                if (!key.isEmpty()) {
                    keys.add(Integer.parseInt(key));
                }
            }
            mComboKeys[i] = new SettingModel();
            mComboKeys[i].name = getString(R.string.combo_key_name, i + 1);
            for (int j = 0; j < mComboKeys[i].keys.length; ++j) {
                mComboKeys[i].keys[j] = new KeyModel();
                mComboKeys[i].keys[j].name = getString(KeyTexts[j]);
                mComboKeys[i].keys[j].enabled = keys.contains(KeyValues[j]);
            }
        }
        mAdapter.bind(mComboKeys);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
        SharedPreferences.Editor editor = prefs.edit();
        for (int i = 0; i < mComboKeys.length; ++i) {
            StringBuilder sb = new StringBuilder();
            for (int j = 0; j < mComboKeys[i].keys.length; ++j) {
                if (mComboKeys[i].keys[j].enabled) {
                    sb.append(KeyValues[j]);
                    sb.append(",");
                }
            }
            if (sb.length() > 0) {
                sb.setLength(sb.length() - 1);
            }
            editor.putString("combo_key_" + i, sb.toString());
        }
        editor.apply();
    }

    static class KeyModel {
        public String name;
        public boolean enabled;
    }

    static class KeyViewHolder extends RecyclerView.ViewHolder implements View.OnClickListener {
        private TextView mKeyName;
        private CheckBox mCheckbox;
        private KeyModel mModel;

        public KeyViewHolder(View itemView) {
            super(itemView);
            mKeyName = itemView.findViewById(R.id.text_setting_name);
            mCheckbox = itemView.findViewById(R.id.checkbox);
            itemView.setOnClickListener(this);
        }

        @Override
        public void onClick(View clicked) {
            mCheckbox.toggle();
            mModel.enabled = mCheckbox.isChecked();
        }

        public void bind(KeyModel model) {
            mModel = model;
            mKeyName.setText(model.name);
            mCheckbox.setChecked(model.enabled);
        }
    }

    static class KeyAdapter extends RecyclerView.Adapter<KeyViewHolder> {
        private KeyModel[] mModels;

        @Override
        public KeyViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View card =
                    LayoutInflater.from(parent.getContext()).inflate(viewType, parent, false);
            return new KeyViewHolder(card);
        }

        @Override
        public void onBindViewHolder(KeyViewHolder holder, int position) {
            KeyModel model = mModels[position];
            holder.bind(model);
        }

        @Override
        public int getItemViewType(int position) {
            return R.layout.list_item_running_checkbox;
        }

        @Override
        public int getItemCount() {
            return mModels == null ? 0 : mModels.length;
        }

        public void bind(KeyModel[] models) {
            mModels = models;
        }
    }

    static class SettingModel {
        String name;
        KeyModel[] keys = new KeyModel[12];
    }

    static class SettingViewHolder extends RecyclerView.ViewHolder {

        private KeyAdapter mAdapter;
        private TextView mName;

        public SettingViewHolder(View itemView) {
            super(itemView);

            mName = itemView.findViewById(R.id.text_setting_name);

            mAdapter = new KeyAdapter();
            RecyclerView listView = itemView.findViewById(R.id.list_keys);
            listView.setAdapter(mAdapter);
            Drawable lineDivider = itemView.getContext().getDrawable(R.drawable.line_divider);
            listView.addItemDecoration(new DividerItemDecoration(lineDivider));
            RecyclerView.LayoutManager layoutManager = new GridLayoutManager(itemView.getContext(), 3);
            listView.setLayoutManager(layoutManager);
        }

        public void bind(SettingModel model) {
            mName.setText(model.name);
            mAdapter.bind(model.keys);
        }
    }

    static class SettingAdapter extends RecyclerView.Adapter<SettingViewHolder> {

        private SettingModel[] mModels;

        @Override
        public SettingViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View card =
                    LayoutInflater.from(parent.getContext()).inflate(viewType, parent, false);
            return new SettingViewHolder(card);
        }

        @Override
        public void onBindViewHolder(SettingViewHolder holder, int position) {
            holder.bind(mModels[position]);
        }

        @Override
        public int getItemViewType(int position) {
            return R.layout.list_item_combo_key;
        }

        @Override
        public int getItemCount() {
            return mModels == null ? 0 : mModels.length;
        }

        public void bind(SettingModel[] models) {
            mModels = models;
            notifyDataSetChanged();
        }
    }
}
