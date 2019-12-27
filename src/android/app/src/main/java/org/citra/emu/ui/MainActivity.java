// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
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
    private static WeakReference<MainActivity> sInstance = new WeakReference<>(null);
    private List<GameFile> mGames;
    private String mDirToAdd;
    private String[] mFilesToAdd;
    private GameAdapter mAdapter;
    private ProgressBar mProgressBar;
    private BroadcastReceiver mBroadcastReceiver;

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
        mGames = new ArrayList<>();
        mAdapter = new GameAdapter();
        RecyclerView GameList = findViewById(R.id.grid_games);
        GameList.setAdapter(mAdapter);
        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        GameList.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(this);
        GameList.setLayoutManager(layoutManager);

        if (PermissionsHandler.checkWritePermission(this)) {
            loadGameList();
            showGames();
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
    }

    @Override
    protected void onDestroy() {
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
                        loadGameList();
                        showGames();
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

    public void refreshLibrary() {
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

    public void saveGameList() {
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
            //
        }

        mGames.clear();
        for (String path : content.split(";")) {
            if (NativeLibrary.isValidFile(path) && new File(path).exists() &&
                NativeLibrary.IsAppExecutable(path)) {
                mGames.add(new GameFile(path));
            }
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
                } else {
                    Toast.makeText(this, "Error: " + name, Toast.LENGTH_LONG).show();
                }
            }
        }
    }

    public void showGames() {
        mGames.sort((GameFile x, GameFile y) -> x.getName().compareTo(y.getName()));
        mAdapter.setGameList(mGames);
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
            mTextCompany.setText(model.getInfo());
            mTextRegion.setText(regions[model.getRegion() + 1]);
            mImageIcon.setImageBitmap(model.getIcon(mImageIcon.getContext()));
        }

        public String getPath() {
            return mModel.getPath();
        }

        public String getProgramId() {
            return mModel.getId();
        }

        public String getTitle() {
            return mModel.getName();
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
            EmulationActivity.launch(view.getContext(), holder.getPath());
        }

        @Override
        public boolean onLongClick(View view) {
            GameViewHolder holder = (GameViewHolder)view.getTag();
            EditorActivity.launch(view.getContext(), holder.getProgramId(), holder.getTitle());
            return true;
        }

        public void setGameList(List<GameFile> games) {
            mGameList = games;
            notifyDataSetChanged();
        }
    }
}
