// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v4.content.LocalBroadcastManager;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.support.v7.widget.Toolbar;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;
import com.nononsenseapps.filepicker.DividerItemDecoration;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.settings.MenuTag;
import org.citra.emu.settings.SettingsActivity;
import org.citra.emu.utils.DirectoryInitialization;
import org.citra.emu.utils.FileBrowserHelper;
import org.citra.emu.utils.PermissionsHandler;

public final class MainActivity extends AppCompatActivity {
    public static final int REQUEST_ADD_DIRECTORY = 1;
    public static final int REQUEST_OPEN_FILE = 2;

    public static final String PREF_LIST_TYPE = "list_type";

    private static WeakReference<MainActivity> sInstance = new WeakReference<>(null);
    private List<GameFile> mGames;
    private String mDirToAdd;
    private String[] mFilesToAdd;
    private GameAdapter mAdapter;
    private ProgressBar mProgressBar;
    private TextView mEmulationInfo;
    private RecyclerView mGameListView;
    private Button mBtnAddFiles;
    private BroadcastReceiver mBroadcastReceiver;
    private boolean mIsListApp;

    public static MainActivity get() {
        return sInstance.get();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        sInstance = new WeakReference<>(this);

        Toolbar toolbar = findViewById(R.id.toolbar_main);
        setSupportActionBar(toolbar);

        mProgressBar = findViewById(R.id.progress_bar);
        mEmulationInfo = findViewById(R.id.emulation_info);
        mBtnAddFiles = findViewById(R.id.btn_add_files);
        mBtnAddFiles.setClickable(true);
        mBtnAddFiles.setOnClickListener(view -> FileBrowserHelper.openDirectoryPicker(this));

        mGames = new ArrayList<>();
        mAdapter = new GameAdapter();
        mGameListView = findViewById(R.id.grid_games);
        mGameListView.setAdapter(mAdapter);
        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        mGameListView.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(this);
        mGameListView.setLayoutManager(layoutManager);

        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
        mIsListApp = pref.getBoolean(PREF_LIST_TYPE, false);

        if (PermissionsHandler.checkWritePermission(this)) {
            showGameList();
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.menu_add_directory:
            FileBrowserHelper.openDirectoryPicker(this);
            return true;

        case R.id.menu_switch_list:
            mIsListApp = !mIsListApp;
            showGameList();
            return true;

        case R.id.menu_settings_core:
            SettingsActivity.launch(this, MenuTag.CONFIG, "");
            return true;

        case R.id.menu_input_binding:
            SettingsActivity.launch(this, MenuTag.INPUT, "");
            return true;

        case R.id.menu_install_cia:
            FileBrowserHelper.openFilePicker(this, REQUEST_OPEN_FILE);
            return true;

        case R.id.menu_refresh:
            refreshLibrary();
            return true;
        }

        return false;
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mDirToAdd != null) {
            if (mIsListApp) {
                mIsListApp = false;
                loadGameList();
            }
            addGamesInDirectory(mDirToAdd);
            mDirToAdd = null;
            saveGameList();
            showGames();
        }

        if (mFilesToAdd != null) {
            List<String> filelist = new ArrayList<>();
            for (String f : mFilesToAdd) {
                if (f.toLowerCase().endsWith(".cia")) {
                    filelist.add(f);
                }
            }
            mFilesToAdd = null;
            final String[] files = filelist.toArray(new String[0]);
            new Thread(() -> NativeLibrary.InstallCIA(files)).start();
        }

        if (mIsListApp) {
            refreshLibrary();
        }
    }

    @Override
    protected void onDestroy() {
        SharedPreferences.Editor editor = PreferenceManager.getDefaultSharedPreferences(this).edit();
        editor.putBoolean(PREF_LIST_TYPE, mIsListApp);
        editor.apply();

        super.onDestroy();
        if (mBroadcastReceiver != null) {
            LocalBroadcastManager.getInstance(this).unregisterReceiver(mBroadcastReceiver);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent result) {
        switch (requestCode) {
        case REQUEST_ADD_DIRECTORY:
            // If the user picked a file, as opposed to just backing out.
            if (resultCode == MainActivity.RESULT_OK) {
                mDirToAdd = FileBrowserHelper.getSelectedDirectory(result);
            }
            break;
        case REQUEST_OPEN_FILE:
            if (resultCode == MainActivity.RESULT_OK) {
                mFilesToAdd = FileBrowserHelper.getSelectedFiles(result);
            }
            break;
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions,
                                           int[] grantResults) {
        switch (requestCode) {
        case PermissionsHandler.REQUEST_CODE_WRITE_PERMISSION:
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                IntentFilter filter = new IntentFilter();
                filter.addAction(DirectoryInitialization.BROADCAST_ACTION);
                mBroadcastReceiver = new BroadcastReceiver() {
                    @Override
                    public void onReceive(Context context, Intent intent) {
                        showGameList();
                    }
                };
                LocalBroadcastManager.getInstance(this).registerReceiver(mBroadcastReceiver,
                                                                         filter);
                DirectoryInitialization.start(this);
            } else {
                Toast.makeText(this, R.string.write_permission_needed, Toast.LENGTH_SHORT).show();
            }
            break;
        default:
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            break;
        }
    }

    public void showGameList() {
        if (mIsListApp) {
            mGames.clear();
            addGamesInDirectoryRecursively(DirectoryInitialization.getSDMCDirectory());
        } else {
            loadGameList();
        }
        showGames();
    }

    public void refreshLibrary() {
        if (mIsListApp) {
            mGames.clear();
            addGamesInDirectoryRecursively(DirectoryInitialization.getSDMCDirectory());
        } else {
            List<String> dirs = new ArrayList<>();
            for (int i = mGames.size(); i > 0; --i) {
                GameFile game = mGames.get(i - 1);
                String path = game.getPath();
                if (new File(path).exists()) {
                    int lastSlash = path.lastIndexOf('/');
                    if (lastSlash == -1)
                        path = "/";
                    else
                        path = path.substring(0, lastSlash);
                    if (!dirs.contains(path)) {
                        dirs.add(path);
                    }
                } else {
                    mGames.remove(i - 1);
                }
            }
            for (String dir : dirs) {
                addGamesInDirectory(dir);
            }
            saveGameList();
        }
        showGames();
    }

    public void addGamesInDirectory(String directory) {
        File[] files = new File(directory).listFiles((File dir, String name) -> {
            if (NativeLibrary.isValidFile(name)) {
                String path = dir.getPath() + File.separator + name;
                for (GameFile game : mGames) {
                    if (path.equals(game.getPath())) {
                        return false;
                    }
                }
                return true;
            }
            return false;
        });

        if (files == null) {
            return;
        }

        for (File f : files) {
            String path = f.getPath();
            if (NativeLibrary.IsAppExecutable(path))
                mGames.add(new GameFile(path));
        }
    }

    public void addGamesInDirectoryRecursively(String root) {
        List<File> dirs = new ArrayList<>();
        dirs.add(new File(root));
        while (dirs.size() > 0) {
            File dir = dirs.get(0);
            File[] files = dir.listFiles();
            dirs.remove(0);
            if (files != null) {
                for (File f : files) {
                    if (f.isDirectory()) {
                        dirs.add(f);
                    } else if (NativeLibrary.isValidFile(f.getName())) {
                        String path = f.getPath();
                        if (NativeLibrary.IsAppExecutable(path)) {
                            mGames.add(new GameFile(path, path.contains("citra-emu/sdmc/Nintendo 3DS")));
                        }
                    }
                }
            }
        }
    }

    public void loadGameList() {
        String content = "";
        File cache = getGameListCache();
        try {
            FileReader reader = new FileReader(cache);
            char[] buffer = new char[(int)cache.length()];
            int size = reader.read(buffer);
            content = new String(buffer);
            reader.close();
        } catch (IOException e) {
            // ignore
        }

        mGames.clear();
        for (String path : content.split(";")) {
            if (NativeLibrary.isValidFile(path) && new File(path).exists() &&
                    NativeLibrary.IsAppExecutable(path)) {
                mGames.add(new GameFile(path));
            }
        }
    }

    public void saveGameList() {
        if (mIsListApp) {
            return;
        }

        StringBuilder sb = new StringBuilder();
        for (GameFile game : mGames) {
            sb.append(game.getPath());
            sb.append(";");
        }

        File cache = getGameListCache();
        FileWriter writer;
        try {
            writer = new FileWriter(cache);
            writer.write(sb.toString());
            writer.close();
        } catch (IOException e) {
            //
        }
    }

    public File getGameListCache() {
        return new File(DirectoryInitialization.getUserDirectory() + File.separator +
                        "gamelist.cache");
    }

    public void updateProgress(String name, int written, int total) {
        if (written < total) {
            mProgressBar.setVisibility(View.VISIBLE);
        } else {
            mProgressBar.setVisibility(View.INVISIBLE);
            if (total == 0) {
                if (written == 0) {
                    Toast.makeText(this, "Install Success!", Toast.LENGTH_LONG).show();
                    if (mIsListApp) {
                        refreshLibrary();
                    }
                } else {
                    Toast.makeText(this, "Error: " + name, Toast.LENGTH_LONG).show();
                }
            }
        }
    }

    public void showGames() {
        if (mIsListApp) {
            mGames.sort((GameFile x, GameFile y) -> {
                if (x.isInstalledApp()) {
                    if (y.isInstalledApp()) {
                        return x.getName().compareTo(y.getName());
                    } else {
                        return -1;
                    }
                } else if (y.isInstalledApp()) {
                    return 1;
                } else {
                    return x.getName().compareTo(y.getName());
                }
            });
        } else {
            mGames.sort((GameFile x, GameFile y) -> x.getName().compareTo(y.getName()));
        }

        if (mGames.size() > 0) {
            mEmulationInfo.setVisibility(View.INVISIBLE);
            mBtnAddFiles.setVisibility(View.INVISIBLE);
            mGameListView.setVisibility(View.VISIBLE);
            mAdapter.setGameList(mGames);
        } else {
            mEmulationInfo.setVisibility(View.VISIBLE);
            mBtnAddFiles.setVisibility(View.VISIBLE);
            mGameListView.setVisibility(View.INVISIBLE);
        }
    }

    static class GameViewHolder extends RecyclerView.ViewHolder {
        private ImageView mImageIcon;
        private TextView mTextTitle;
        private TextView mTextRegion;
        private TextView mTextCompany;
        private GameFile mModel;

        public GameViewHolder(View itemView) {
            super(itemView);
            itemView.setTag(this);
            mImageIcon = itemView.findViewById(R.id.image_game_screen);
            mTextTitle = itemView.findViewById(R.id.text_game_title);
            mTextRegion = itemView.findViewById(R.id.text_region);
            mTextCompany = itemView.findViewById(R.id.text_company);
        }

        public void bind(GameFile model) {
            int[] regions = {
                R.string.region_invalid, R.string.region_japan,     R.string.region_north_america,
                R.string.region_europe,  R.string.region_australia, R.string.region_china,
                R.string.region_korea,   R.string.region_taiwan,
            };
            mModel = model;
            mTextTitle.setText(model.getName());
            if ("0004000000000000".equals(model.getId())) {
                mTextTitle.setTextColor(Color.RED);
            } else {
                mTextTitle.setTextColor(Color.BLACK);
            }
            mTextCompany.setText(model.getInfo());
            if (model.isInstalled()) {
                String region = mTextRegion.getContext().getString(regions[model.getRegion() + 1]);
                String desc = getAppDesc(model);
                mTextRegion.setText(region + desc);
            } else {
                mTextRegion.setText(regions[model.getRegion() + 1]);
            }
            mImageIcon.setImageBitmap(model.getIcon(mImageIcon.getContext()));
        }

        public String getAppDesc(GameFile model) {
            String path = model.getPath();
            File appFile = new File(path);
            if (appFile.exists()) {
                String size = NativeLibrary.Size2String(appFile.length());
                String desc = size + (model.isInstalledApp() ? " APP" : " DLC");
                return mTextRegion.getContext().getString(R.string.installed_app, desc);
            } else {
                return "";
            }
        }

        public GameFile getModel() {
            return mModel;
        }
    }

    static class GameAdapter extends RecyclerView.Adapter<GameViewHolder>
        implements View.OnClickListener, View.OnLongClickListener {
        private List<GameFile> mGameList = new ArrayList<>();

        @Override
        public GameViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View gameCard =
                LayoutInflater.from(parent.getContext()).inflate(viewType, parent, false);
            gameCard.setOnClickListener(this);
            gameCard.setOnLongClickListener(this);
            return new GameViewHolder(gameCard);
        }

        @Override
        public void onBindViewHolder(GameViewHolder holder, int position) {
            holder.bind(mGameList.get(position));
        }

        @Override
        public int getItemViewType(int position) {
            return R.layout.card_game;
        }

        @Override
        public int getItemCount() {
            return mGameList.size();
        }

        @Override
        public void onClick(View view) {
            GameViewHolder holder = (GameViewHolder)view.getTag();
            GameFile model = holder.getModel();
            if (!model.isInstalledDLC()) {
                EmulationActivity.launch(view.getContext(), model);
            }
        }

        @Override
        public boolean onLongClick(View view) {
            GameViewHolder holder = (GameViewHolder)view.getTag();
            GameFile model = holder.getModel();
            EditorActivity.launch(view.getContext(), model.getId(), model.getName());
            return true;
        }

        public void setGameList(List<GameFile> games) {
            mGameList = games;
            notifyDataSetChanged();
        }
    }
}
