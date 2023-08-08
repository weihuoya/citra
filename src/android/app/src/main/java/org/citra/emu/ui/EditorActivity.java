// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.text.Editable;
import android.text.Spanned;
import android.text.TextUtils;
import android.text.TextWatcher;
import android.text.style.ForegroundColorSpan;
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
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.pm.ShortcutManagerCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import androidx.appcompat.widget.Toolbar;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.settings.MenuTag;
import org.citra.emu.settings.SettingsActivity;
import org.citra.emu.utils.CitraDirectory;

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
        private ImageButton mBtnDelete;

        public CheatEntryViewHolder(View itemView) {
            super(itemView);
            mTextName = itemView.findViewById(R.id.text_setting_name);
            mTextDescription = itemView.findViewById(R.id.text_setting_description);
            mCheckbox = itemView.findViewById(R.id.checkbox);
            mBtnDelete = itemView.findViewById(R.id.btn_delete);
            mBtnDelete.setOnClickListener(view -> {
                deleteCheatAt(getBindingAdapterPosition());
            });
            itemView.setOnClickListener(this);
        }

        public void bind(CheatEntry entry) {
            mModel = entry;
            mTextName.setText(entry.getName());
            mTextDescription.setText(entry.getInfo());
            if (mIsManageCheats) {
                itemView.setClickable(false);
                mBtnDelete.setVisibility(View.VISIBLE);
                mCheckbox.setVisibility(View.INVISIBLE);
            } else {
                itemView.setClickable(true);
                mBtnDelete.setVisibility(View.INVISIBLE);
                mCheckbox.setChecked(entry.enabled);
                mCheckbox.setVisibility(entry.codes.size() > 0 ? View.VISIBLE : View.INVISIBLE);
            }
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
            View itemView = inflater.inflate(R.layout.list_item_editor_checkbox, parent, false);
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
    private boolean mCancelSave;
    private EditText mEditor;
    private Button mBtnConfirm;
    private Button mBtnDone;
    private RecyclerView mListView;
    private CheatEntryAdapter mAdapter;
    private List<CheatEntry> mCheats;
    private boolean mIsManageCheats;

    public static void launch(Context context, GameFile game) {
        Intent settings = new Intent(context, EditorActivity.class);
        settings.putExtra(EXTRA_GAME_ID, game.getId());
        settings.putExtra(EXTRA_GAME_NAME, game.getName());
        settings.putExtra(EXTRA_GAME_PATH, game.getPath());
        context.startActivity(settings);
    }

    @SuppressLint("ClickableViewAccessibility")
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

        mIsManageCheats = false;
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
                int length = s.length();
                int line_begin = -1;
                int line_status = 0;
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < length; ++i) {
                    char c = s.charAt(i);
                    if (line_begin == -1) {
                        if (c == '*') {
                            line_status = 1;
                        } else if (c == '[') {
                            line_status = 2;
                        } else if (c == '/') {
                            line_status = 3;
                        }
                        line_begin = i;
                    }
                    if (c == '\n' || c == '\r') {
                        if (line_status == 1) {
                            int color = Color.GRAY;
                            if (sb.length() == CHEAT_ENABLED_TEXT.length() && sb.toString().equals(CHEAT_ENABLED_TEXT)) {
                                color = Color.MAGENTA;
                            }
                            s.setSpan(new ForegroundColorSpan(color), line_begin, i, Spanned.SPAN_EXCLUSIVE_EXCLUSIVE);
                        } else if (line_status == 2) {
                            s.setSpan(new ForegroundColorSpan(Color.BLUE), line_begin, i, Spanned.SPAN_EXCLUSIVE_EXCLUSIVE);
                        } else if (line_status == 3) {
                            s.setSpan(new ForegroundColorSpan(Color.GRAY), line_begin, i, Spanned.SPAN_EXCLUSIVE_EXCLUSIVE);
                        }
                        line_status = 0;
                        line_begin = -1;
                        sb.setLength(0);
                    } else {
                        sb.append(c);
                    }
                }
            }
        });

        mAdapter = new CheatEntryAdapter();
        mListView.setAdapter(mAdapter);
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

        mCancelSave = false;
        Button buttonCancel = findViewById(R.id.button_cancel);
        buttonCancel.setOnClickListener(view -> {
            mCancelSave = true;
            finish();
        });

        mBtnConfirm = findViewById(R.id.button_confirm);
        mBtnConfirm.setOnClickListener(view -> {
            toggleListView(mEditor.getVisibility() == View.VISIBLE);
        });

        mBtnDone = findViewById(R.id.button_done);
        mBtnDone.setOnClickListener(view -> toggleManageCheats());

        loadCheatFile(mGameId);
        toggleListView(mCheats.size() > 0);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (!mCancelSave) {
            saveCheatCode(mGameId);
        }
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
        boolean visible = isMiUiSystem() && CitraDirectory.isExternalStorageLegacy();
        inflater.inflate(R.menu.menu_editor, menu);
        menu.findItem(R.id.menu_open_archive).setVisible(visible);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_settings_core:
                SettingsActivity.launch(this, MenuTag.CONFIG, mGameId, mGameName);
                return true;

            case R.id.menu_open_archive:
                jumpToExplore();
                return true;

            case R.id.menu_manage_cheats:
                toggleManageCheats();
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
            mBtnConfirm.setText(R.string.edit_cheat);
        } else {
            mListView.setVisibility(View.INVISIBLE);
            mEditor.setVisibility(View.VISIBLE);
            // reload
            mEditor.setText(loadCheatText());
            mBtnConfirm.setText(R.string.cheat_list);
        }
    }

    private void toggleManageCheats() {
        if (mIsManageCheats) {
            mIsManageCheats = false;
            mAdapter.loadCheats(mCheats);
            mBtnDone.setVisibility(View.INVISIBLE);
        } else {
            mIsManageCheats = true;
            if (mEditor.getVisibility() == View.VISIBLE) {
                toggleListView(true);
            } else {
                mAdapter.loadCheats(mCheats);
            }
            mBtnDone.setVisibility(View.VISIBLE);
        }
    }

    private void deleteCheatAt(int position) {
        if (position >= 0 && position < mCheats.size()) {
            mCheats.remove(position);
            mAdapter.notifyItemRemoved(position);
        }
    }

    private void jumpToExplore() {
        String root;
        String pid = mGameId.substring(0, 8).toLowerCase();
        String subid = mGameId.substring(8).toLowerCase();
        if (pid.equals("00040010") || pid.equals("00040030")) {
            root = CitraDirectory.getSystemTitleDirectory();
        } else {
            root = CitraDirectory.getApplicationDirectory();
        }
        String path = String.format("%s/%s/%s", root, pid, subid);

        try {
            Intent intent = new Intent("miui.intent.action.OPEN");
            intent.addFlags(0x10000000);
            if (isMiUiInternational()) {
                intent.setPackage("com.mi.android.globalFileexplorer");
            } else {
                intent.setPackage("com.android.fileexplorer");
            }
            intent.putExtra("explorer_path", path);
            startActivity(intent);
        } catch (Exception e) {
            // ignore
        }
    }

    private boolean isMiUiSystem() {
        try {
            Class<?> c = Class.forName("android.os.SystemProperties");
            Method get = c.getMethod("get", String.class);
            String miuiName = (String) get.invoke(null, "ro.miui.ui.version.name");
            return !TextUtils.isEmpty(miuiName);
        } catch (Exception e) {
            // ignore
        }
        return false;
    }

    private boolean isMiUiInternational() {
        try {
            Class<?> buildClazz = Class.forName("miui.os.Build");
            Field isInternational = buildClazz.getDeclaredField("IS_INTERNATIONAL_BUILD");
            return isInternational.getBoolean(null);
        } catch (Exception e) {
            // ignore
        }
        return false;
    }

    private void deleteShaderCache() {
        String dir = CitraDirectory.getUserDirectory() + "/cache/";
        File cache = new File(dir + mGameId + ".cache");
        File shader = new File(dir + mGameId + ".shader");
        File meta = new File(dir + mGameId + ".shader.meta");
        if (cache.exists()) {
            shader.delete();
            meta.delete();
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
                String system = CitraDirectory.getSystemTitleDirectory();
                String path = String.format("%s/%s/%s", system, pid, subid);
                CitraDirectory.deleteAllFiles(path);
            } else if (pid.equals("00040000")) {
                // App
                String root = CitraDirectory.getApplicationDirectory();
                String path = String.format("%s/%s/%s/content", root, pid, subid);
                CitraDirectory.deleteAllFiles(path);
                // Updates
                path = String.format("%s/0004000e/%s/content", root, subid);
                CitraDirectory.deleteAllFiles(path);
                // DLCs
                path = String.format("%s/0004008c/%s/content", root, subid);
                CitraDirectory.deleteAllFiles(path);
            } else if (pid.equals("0004000e") || pid.equals("0004008c")) {
                // Updates
                String root = CitraDirectory.getApplicationDirectory();
                String path = String.format("%s/0004000e/%s/content", root, subid);
                CitraDirectory.deleteAllFiles(path);
                // DLCs
                path = String.format("%s/0004008c/%s/content", root, subid);
                CitraDirectory.deleteAllFiles(path);
            }
            Toast.makeText(this, "Delete Success!", Toast.LENGTH_LONG).show();
        });
        builder.setNegativeButton(android.R.string.no, null);
        builder.show();
    }

    private void loadCheatFile(String programId) {
        File cheatFile = CitraDirectory.getCheatFile(programId);
        mCheats.clear();
        if (cheatFile == null || !cheatFile.exists()) {
            String code = getBuiltinCheat(programId);
            loadCheatCode(code);
            if (code.contains(CHEAT_ENABLED_TEXT)) {
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
            line = line.trim();
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
                    entry.codes.add(validateCheat(line));
                }
            }
        }

        if (entry.infos.size() > 0 || entry.codes.size() > 0) {
            mCheats.add(entry);
        }
    }

    private String validateCheat(String line) {
        StringBuilder sb = new StringBuilder();
        boolean insertSpace = false;
        for (int i = 0; i < line.length(); ++i) {
            char c = line.charAt(i);
            if (c >= '0' && c <= '9') {
                sb.append(c);
                insertSpace = true;
            } else if (c >= 'a' && c <= 'z') {
                sb.append(Character.toUpperCase(c));
                insertSpace = true;
            } else if (c >= 'A' && c <= 'Z') {
                sb.append(c);
                insertSpace = true;
            } else if (Character.isWhitespace(c)) {
                if (insertSpace) {
                    sb.append(' ');
                    insertSpace = false;
                }
            } else {
                // invalid cheat code
                sb.append(line.substring(i));
                break;
            }
        }
        if (sb.length() > 0 && sb.charAt(sb.length() - 1) == ' ') {
            sb.deleteCharAt(sb.length() - 1);
        }
        return sb.toString();
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
        File cheatFile = CitraDirectory.getCheatFile(programId);
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
}
