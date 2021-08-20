// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.pm.ShortcutManagerCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import androidx.appcompat.widget.Toolbar;

import com.nononsenseapps.filepicker.DividerItemDecoration;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.utils.DirectoryInitialization;

public final class EditorActivity extends AppCompatActivity {
    public static final String EXTRA_GAME_ID = "GameId";
    public static final String EXTRA_GAME_NAME = "GameName";
    public static final String EXTRA_GAME_PATH = "GamePath";
    private static final String CHEAT_ENABLED_TEXT = "*citra_enabled";

    static class CheatEntry {
        boolean enabled = false;
        public List<String> infos = new ArrayList<>();
        public List<String> codes = new ArrayList<>();

        public String getName() {
            if (infos.size() > 0) {
                return infos.get(0);
            } else {
                return "Cheat";
            }
        }

        public String getInfo() {
            StringBuilder sb = new StringBuilder();
            if (infos.size() > 1) {
                for (int i = 1; i < infos.size(); ++i) {
                    sb.append(infos.get(i));
                }
            }
            return sb.toString();
        }
    }

    class CheatEntryViewHolder extends RecyclerView.ViewHolder implements View.OnClickListener {
        private CheatEntry mModel;
        private TextView mTextName;
        private TextView mTextDescription;
        private CheckBox mCheckbox;

        public CheatEntryViewHolder(View itemView) {
            super(itemView);
            mTextName = itemView.findViewById(R.id.text_setting_name);
            mTextDescription = itemView.findViewById(R.id.text_setting_description);
            mCheckbox = itemView.findViewById(R.id.checkbox);
            itemView.setOnClickListener(this);
        }

        public void bind(CheatEntry entry) {
            mModel = entry;
            mTextName.setText(entry.getName());
            mTextDescription.setText(entry.getInfo());
            mCheckbox.setChecked(entry.enabled);
            mCheckbox.setVisibility(entry.codes.size() > 0 ? View.VISIBLE : View.INVISIBLE);
        }

        @Override
        public void onClick(View v) {
            if (mModel.codes.size() > 0) {
                mModel.enabled = !mModel.enabled;
                mCheckbox.setChecked(mModel.enabled);
            } else {
                mModel.enabled = false;
            }
        }
    }

    class CheatEntryAdapter extends RecyclerView.Adapter<CheatEntryViewHolder> {
        private List<CheatEntry> mDataset;

        @NonNull
        @Override
        public CheatEntryViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            LayoutInflater inflater = LayoutInflater.from(parent.getContext());
            View itemView = inflater.inflate(R.layout.list_item_setting_checkbox, parent, false);
            return new CheatEntryViewHolder(itemView);
        }

        @Override
        public int getItemCount() {
            return mDataset != null ? mDataset.size() : 0;
        }

        @Override
        public void onBindViewHolder(@NonNull CheatEntryViewHolder holder, int position) {
            holder.bind(mDataset.get(position));
        }

        public void loadCheats(List<CheatEntry> list) {
            mDataset = list;
            notifyDataSetChanged();
        }
    }

    private String mGameId;
    private String mGameName;
    private String mGamePath;
    private boolean mReloadText;
    private EditText mEditor;
    private RecyclerView mListView;
    private CheatEntryAdapter mAdapter;
    private List<CheatEntry> mCheats;

    public static void launch(Context context, GameFile game) {
        Intent settings = new Intent(context, EditorActivity.class);
        settings.putExtra(EXTRA_GAME_ID, game.getId());
        settings.putExtra(EXTRA_GAME_NAME, game.getName());
        settings.putExtra(EXTRA_GAME_PATH, game.getPath());
        context.startActivity(settings);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_editor);

        if (savedInstanceState == null) {
            Intent intent = getIntent();
            mGameId = intent.getStringExtra(EXTRA_GAME_ID);
            mGameName = intent.getStringExtra(EXTRA_GAME_NAME);
            mGamePath = intent.getStringExtra(EXTRA_GAME_PATH);
        } else {
            mGameId = savedInstanceState.getString(EXTRA_GAME_ID);
            mGameName = savedInstanceState.getString(EXTRA_GAME_NAME);
            mGamePath = savedInstanceState.getString(EXTRA_GAME_PATH);
        }

        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        setTitle(mGameName);

        mCheats = new ArrayList<>();

        TextView gameInfo = findViewById(R.id.game_info);
        gameInfo.setText("ID: " + mGameId);

        mEditor = findViewById(R.id.code_content);
        mListView = findViewById(R.id.code_list);

        mReloadText = false;
        mEditor.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                mReloadText = true;
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        mAdapter = new CheatEntryAdapter();
        mListView.setAdapter(mAdapter);
        mListView.addItemDecoration(new DividerItemDecoration(lineDivider));
        mListView.setLayoutManager(new LinearLayoutManager(this));

        Button buttonShortcut = findViewById(R.id.button_shortcut);
        if (ShortcutManagerCompat.isRequestPinShortcutSupported(this)) {
            buttonShortcut.setVisibility(View.VISIBLE);
        } else {
            buttonShortcut.setVisibility(View.INVISIBLE);
        }
        buttonShortcut.setOnClickListener(view -> {
            ShortcutDialog.newInstance(mGamePath).show(getSupportFragmentManager(), "ShortcutDialog");
        });

        Button buttonConfirm = findViewById(R.id.button_confirm);
        buttonConfirm.setOnClickListener(view -> {
            saveCheatCode(mGameId);
            mEditor.clearFocus();
            finish();
        });

        Button buttonCancel = findViewById(R.id.button_cancel);
        buttonCancel.setOnClickListener(view -> {
            mEditor.clearFocus();
            finish();
        });

        loadCheatFile(mGameId);
        toggleListView(mCheats.size() > 0);
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        outState.putString(EXTRA_GAME_ID, mGameId);
        outState.putString(EXTRA_GAME_NAME, mGameName);
        outState.putString(EXTRA_GAME_PATH, mGamePath);
        super.onSaveInstanceState(outState);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_editor, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_toggle_list:
                toggleListView(mEditor.getVisibility() == View.VISIBLE);
                return true;

            case R.id.menu_delete_sdmc:
                deleteAppSdmc();
                return true;

            case R.id.menu_delete_shader_cache:
                deleteShaderCache();
                return true;
        }

        return false;
    }

    private void toggleListView(boolean isShowList) {
        if (isShowList) {
            InputMethodManager imm = getSystemService(InputMethodManager.class);
            imm.hideSoftInputFromWindow(getWindow().getDecorView().getWindowToken(), 0);
            mListView.setVisibility(View.VISIBLE);
            mEditor.setVisibility(View.INVISIBLE);
            // reload
            if (mReloadText) {
                mCheats.clear();
                loadCheatCode(mEditor.getText().toString());
                mReloadText = false;
            }
            mAdapter.loadCheats(mCheats);
        } else {
            mListView.setVisibility(View.INVISIBLE);
            mEditor.setVisibility(View.VISIBLE);
            // reload
            mEditor.setText(loadCheatText());
        }
    }

    private void deleteShaderCache() {
        File cache = DirectoryInitialization.getShaderCacheFile(mGameId);
        if (cache.exists()) {
            if (cache.delete()) {
                Toast.makeText(this, R.string.delete_success, Toast.LENGTH_SHORT).show();
            }
        }
    }

    private void deleteAppSdmc() {
        AlertDialog.Builder builder = new AlertDialog.Builder(EditorActivity.this);
        builder.setMessage(R.string.delete_confirm_notice);
        builder.setPositiveButton(android.R.string.ok, (DialogInterface dialog, int which) -> {
            String pid = mGameId.substring(0, 8).toLowerCase();
            String subid = mGameId.substring(8).toLowerCase();
            if (pid.equals("00040010") || pid.equals("00040030")) {
                String system = DirectoryInitialization.getSystemTitleDirectory();
                String path = system + "/" + pid + "/" + subid;
                deleteContents(new File(path));
            } else if (pid.equals("00040000")) {
                String root = DirectoryInitialization.getApplicationDirectory();
                String path = root + "/" + pid + "/" + subid;
                deleteContents(new File(path));

                // DLC
                path = root + "/0004000e/" + subid;
                deleteContents(new File(path));

                // DLC
                path = root + "/0004008c/" + subid;
                deleteContents(new File(path));
            } else if (pid.equals("0004000e")) {
                // DLC
                String root = DirectoryInitialization.getApplicationDirectory();
                String path = root + "/" + pid + "/" + subid;
                deleteContents(new File(path));
            }
            Toast.makeText(this, "Delete Success!", Toast.LENGTH_LONG).show();
        });
        builder.setNegativeButton(android.R.string.no, null);
        builder.show();
    }

    private void loadCheatFile(String programId) {
        File cheatFile = DirectoryInitialization.getCheatFile(programId);
        mCheats.clear();
        if (cheatFile == null || !cheatFile.exists()) {
            String code = getBuiltinCheat(programId);
            loadCheatCode(code);
            if (code.contains("*citra_enabled")) {
                saveCheatCode(programId);
            }
            return;
        }

        StringBuilder sb = new StringBuilder();
        try {
            BufferedReader reader = new BufferedReader(new FileReader(cheatFile));
            String line = reader.readLine();
            while (line != null) {
                sb.append(line.trim());
                sb.append(System.lineSeparator());
                line = reader.readLine();
            }
            reader.close();
        } catch (IOException e) {
            //
        }

        loadCheatCode(sb.toString());
    }

    private String getBuiltinCheat(String programId) {
        StringBuilder sb = new StringBuilder();
        try {
            byte[] buffer = new byte[4096];
            InputStream inputStream = getAssets().open("cheats/" + programId + ".txt");
            int length = inputStream.read(buffer);
            if (length > 0) {
                sb.append(new String(buffer, 0, length));
            }
        } catch (IOException e) {
            // ignore;
        }
        return sb.toString();
    }

    private void loadCheatCode(String data) {
        String[] lines = data.split(System.lineSeparator());
        CheatEntry entry = new CheatEntry();
        for (String line : lines) {
            if (!line.isEmpty()) {
                if (line.charAt(0) == '[') {
                    if (entry.infos.size() > 0 || entry.codes.size() > 0) {
                        mCheats.add(entry);
                        entry = new CheatEntry();
                    }
                    entry.infos.add(line);
                } else if (line.charAt(0) == '*') {
                    if (CHEAT_ENABLED_TEXT.equals(line)) {
                        entry.enabled = true;
                    } else {
                        entry.infos.add(line);
                    }
                } else {
                    entry.codes.add(line);
                }
            }
        }

        if (entry.infos.size() > 0 || entry.codes.size() > 0) {
            mCheats.add(entry);
        }
    }

    private String loadCheatText() {
        StringBuilder sb = new StringBuilder();
        for(CheatEntry entry : mCheats) {
            for (String info : entry.infos) {
                sb.append(info);
                sb.append(System.lineSeparator());
            }
            if (entry.enabled) {
                sb.append(CHEAT_ENABLED_TEXT);
                sb.append(System.lineSeparator());
            }
            for (String code : entry.codes) {
                sb.append(code);
                sb.append(System.lineSeparator());
            }
            sb.append(System.lineSeparator());
        }
        return sb.toString();
    }

    private void saveCheatCode(String programId) {
        File cheatFile = DirectoryInitialization.getCheatFile(programId);
        String content = mReloadText ? mEditor.getText().toString() : loadCheatText();
        if (content.isEmpty()) {
            cheatFile.delete();
        } else {
            try {
                FileWriter writer = new FileWriter(cheatFile);
                writer.write(content);
                writer.close();
            } catch (IOException e) {
                //
            }
        }
    }

    public static boolean deleteContents(File dir) {
        File[] files = dir.listFiles();
        boolean success = true;
        if (files != null) {
            for (File file : files) {
                if (file.isDirectory()) {
                    success &= deleteContents(file);
                }
                if (!file.delete()) {
                    success = false;
                }
            }
        }
        return success;
    }
}
