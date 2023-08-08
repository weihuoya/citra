// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.annotation.SuppressLint;
import android.content.ClipData;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.UriPermission;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.view.LayoutInflater;
import android.view.View;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.documentfile.provider.DocumentFile;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentActivity;
import androidx.preference.PreferenceManager;
import androidx.swiperefreshlayout.widget.SwipeRefreshLayout;
import androidx.viewpager2.adapter.FragmentStateAdapter;
import androidx.viewpager2.widget.ViewPager2;

import com.google.android.material.appbar.MaterialToolbar;
import com.google.android.material.tabs.TabLayout;
import com.google.android.material.tabs.TabLayoutMediator;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.settings.MenuTag;
import org.citra.emu.settings.SettingsActivity;
import org.citra.emu.utils.CitraDirectory;
import org.citra.emu.utils.FileBrowserHelper;
import org.citra.emu.utils.GZipUtil;
import org.citra.emu.utils.PermissionsHandler;

public final class MainActivity extends AppCompatActivity {
    public static final int PAGE_FRAGMENT_EXTERNAL = 0;
    public static final int PAGE_FRAGMENT_INSTALLED = 1;
    public static final int PAGE_FRAGMENT_CONTENT = 2;

    public static final String PREF_TAB_INDEX = "tab_index";

    @SuppressLint("StaticFieldLeak")
    private class RefreshTask extends AsyncTask<Boolean, Void, Void> {
        private final HashSet<String> dirSet = new HashSet<>();
        private final HashSet<Uri> uriSet = new HashSet<>();
        private final List<GameFile> externals = new ArrayList<>();
        private final List<GameFile> installs = new ArrayList<>();
        private final List<GameFile> contents = new ArrayList<>();
        private final Context context;
        private final boolean isInstalled;

        public RefreshTask(boolean isInstalled) {
            this.context = null;
            this.isInstalled = isInstalled;
        }

        public RefreshTask(String dir) {
            this.context = null;
            isInstalled = false;
            dirSet.add(dir);
        }

        public RefreshTask(Context context, List<Uri> list) {
            this.context = context;
            isInstalled = false;
            uriSet.addAll(list);
        }

        @Override
        protected Void doInBackground(Boolean... args) {
            boolean cleanCache = args[0];
            if (cleanCache) {
                CitraDirectory.clearIconCache();
            }
            if (context != null) {
                refreshSaf();
            } else {
                refreshLegacy();
            }
            saveGameList();
            return null;
        }

        protected void refreshLegacy() {
            List<File> dirs = new ArrayList<>();
            if (isInstalled) {
                dirs.add(new File(CitraDirectory.getSystemApplicationDirectory()));
                dirs.add(new File(CitraDirectory.getSystemAppletDirectory()));
                dirs.add(new File(CitraDirectory.getSDMCDirectory()));
            } else {
                for (GameFile game : externals) {
                    String path = game.getPath();
                    if (new File(path).isFile()) {
                        int lastSlash = path.lastIndexOf('/');
                        if (lastSlash == -1) {
                            path = "/";
                        } else {
                            path = path.substring(0, lastSlash);
                        }
                        dirSet.add(path);
                    }
                }
                for (String dir : dirSet) {
                    dirs.add(new File(dir));
                }
                externals.clear();
            }
            refreshDirectoryLegacy(dirs);
        }

        protected void refreshDirectoryLegacy(List<File> dirs) {
            while (dirs.size() > 0) {
                File dir = dirs.get(0);
                File[] files = dir.listFiles();
                dirs.remove(0);
                if (files != null) {
                    for (File f : files) {
                        String path = f.getPath();
                        if (f.isDirectory()) {
                            // recursive search
                            if (isInstalled || !dirSet.contains(path)) {
                                dirs.add(f);
                            }
                        } else if (NativeLibrary.isValidFile(path)) {
                            GameFile game = new GameFile(path, isInstalled);
                            if (isInstalled) {
                                if (game.isInstalledDLC() && NativeLibrary.IsAppVisible(path)) {
                                    game.init();
                                    contents.add(game);
                                } else if (NativeLibrary.IsAppExecutable(path)) {
                                    game.init();
                                    installs.add(game);
                                }
                            } else if (NativeLibrary.IsAppExecutable(path)) {
                                game.init();
                                externals.add(game);
                            }
                        }
                    }
                }
            }

            if (isInstalled) {
                contents.sort(Comparator.comparing(GameFile::getName));
                installs.sort(Comparator.comparing(GameFile::getName));
            } else {
                externals.sort(Comparator.comparing(GameFile::getName));
            }
        }

        protected void refreshSaf() {
            List<Uri> dirs = new ArrayList<>(uriSet);
            while (dirs.size() > 0) {
                Uri dirUri = dirs.get(0);
                dirs.remove(0);
                DocumentFile documentFile = DocumentFile.fromTreeUri(context, dirUri);
                if (documentFile == null) {
                    continue;
                }
                DocumentFile[] files = documentFile.listFiles();
                for (DocumentFile file : files) {
                    Uri uri = file.getUri();
                    String path = uri.toString();
                    if (file.isDirectory()) {
                        if (!uriSet.contains(uri)) {
                            dirs.add(uri);
                        }
                    } else if (NativeLibrary.isValidFile(path) && NativeLibrary.IsAppExecutable(path)) {
                        GameFile game = new GameFile(path, false);
                        game.init();
                        externals.add(game);
                    }
                }
            }

            externals.sort(Comparator.comparing(GameFile::getName));
        }

        protected void saveGameList() {
            StringBuilder sb = new StringBuilder();

            for (GameFile game : externals) {
                sb.append(game.getPath());
                sb.append(";");
            }

            sb.append("\n");
            for (GameFile game : installs) {
                sb.append(game.getPath());
                sb.append(";");
            }

            sb.append("\n");
            for (GameFile game : contents) {
                sb.append(game.getPath());
                sb.append(";");
            }

            File cache = CitraDirectory.getGameListFile();
            GZipUtil.writeAllText(cache, sb.toString());
        }

        @Override
        protected void onPreExecute() {
            mSwipeRefresh.setRefreshing(true);
            // for save game list
            if (context == null) {
                externals.addAll(mExternalGames);
            }
            if (!isInstalled) {
                installs.addAll(mInstalledGames);
                contents.addAll(mInstalledContents);
            }
        }

        @Override
        protected void onPostExecute(Void args) {
            mExternalGames = externals;
            mInstalledGames = installs;
            mInstalledContents = contents;
            if (isInstalled) {
                refreshInstalledContents();
                refreshInstalledGames();
            } else {
                refreshExternalGames();
            }
            mSwipeRefresh.setRefreshing(false);
        }
    }

    @SuppressLint("StaticFieldLeak")
    private class InstallTask extends AsyncTask<Void, Void, Void> {
        private final String[] files;

        public InstallTask(List<String> files) {
            this.files = files.toArray(new String[0]);
        }

        @Override
        protected void onPreExecute() {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }

        @Override
        protected void onPostExecute(Void unused) {
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }

        @Override
        protected Void doInBackground(Void... voids) {
            NativeLibrary.InstallCIA(files);
            return null;
        }
    }

    @SuppressLint("StaticFieldLeak")
    private class LoadGameListTask extends AsyncTask<Void, Void, Void> {
        List<GameFile> externals = new ArrayList<>();
        List<GameFile> installs = new ArrayList<>();
        List<GameFile> contents = new ArrayList<>();

        @Override
        protected void onPreExecute() {
            mSwipeRefresh.setRefreshing(true);
        }

        @Override
        protected void onPostExecute(Void unused) {
            mSwipeRefresh.setRefreshing(false);
            mExternalGames = externals;
            mInstalledGames = installs;
            mInstalledContents = contents;
            showGameList();
        }

        @Override
        protected Void doInBackground(Void... voids) {
            loadGameList();
            return null;
        }

        protected void loadGameList() {
            String content;
            File cache = CitraDirectory.getGameListFile();
            if (GZipUtil.isGZipped(cache)) {
                content = GZipUtil.readAllText(cache);
            } else {
                try {
                    FileReader reader = new FileReader(cache);
                    char[] buffer = new char[(int)cache.length()];
                    int size = reader.read(buffer);
                    content = new String(buffer, 0, size);
                    reader.close();
                } catch (IOException e) {
                    return;
                }
            }

            String[] lines = content.split("\n");
            if (lines.length > 0) {
                for (String path : lines[0].split(";")) {
                    if (path.startsWith("content://") || new File(path).exists()) {
                        GameFile game = new GameFile(path, false);
                        game.init();
                        externals.add(game);
                    }
                }
            }

            if (lines.length > 1) {
                for (String path : lines[1].split(";")) {
                    if (new File(path).exists()) {
                        GameFile game = new GameFile(path, true);
                        game.init();
                        installs.add(game);
                    }
                }
            }

            if (lines.length > 2) {
                for (String path : lines[2].split(";")) {
                    if (new File(path).exists()) {
                        GameFile game = new GameFile(path, true);
                        game.init();
                        contents.add(game);
                    }
                }
            }
        }
    }

    private static WeakReference<MainActivity> sInstance = new WeakReference<>(null);

    private String mDirToAdd;
    private List<String> mFilesToAdd;
    private boolean mRefreshPersistedUri;
    private ProgressBar mProgressBar;
    private TextView mProgressText;
    private TabLayout mTabLayout;
    private TabLayoutMediator mMediator;
    private MainPagerAdapter mPagerAdapter;
    private ViewPager2 mViewPager;
    private SwipeRefreshLayout mSwipeRefresh;
    private int mTabIndex;

    private Handler mHandler;
    private final boolean[] mRunningHandlers = new boolean[] {false, false, false};

    private boolean mIsLoadGameList;
    private List<GameFile> mExternalGames = new ArrayList<>();
    private List<GameFile> mInstalledGames = new ArrayList<>();
    private List<GameFile> mInstalledContents = new ArrayList<>();

    public static MainActivity get() {
        return sInstance.get();
    }

    @SuppressLint("NonConstantResourceId")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        sInstance = new WeakReference<>(this);
        mHandler = new Handler(getMainLooper());

        if (PermissionsHandler.checkWritePermission(this)) {
            CitraDirectory.start(this);
        }

        MaterialToolbar toolbar = findViewById(R.id.top_appbar);
        toolbar.setOnMenuItemClickListener(menuItem -> {
            switch (menuItem.getItemId()) {
                case R.id.menu_add_directory:
                    FileBrowserHelper.openDirectoryPicker(this);
                    return true;

                case R.id.menu_settings_core:
                    SettingsActivity.launch(this, MenuTag.CONFIG, "", "");
                    return true;

                case R.id.menu_input_binding:
                    SettingsActivity.launch(this, MenuTag.INPUT, "", "");
                    return true;

                case R.id.menu_combo_key:
                    ComboKeyActivity.launch(this);
                    return true;

                case R.id.menu_install_cia:
                    FileBrowserHelper.openFilePicker(this);
                    return true;

                case R.id.menu_multiplayer:
                    RunningSettingDialog dialog = RunningSettingDialog.newInstance(RunningSettingDialog.MENU_MULTIPLAYER);
                    dialog.show(getSupportFragmentManager(), "RunningSettingDialog");
                    return true;

                case R.id.menu_refresh:
                    refreshLibrary();
                    return true;
            }
            return false;
        });

        mProgressBar = findViewById(R.id.progress_bar);
        mProgressText = findViewById(R.id.progress_text);
        mTabLayout = findViewById(R.id.tabs_main);
        mViewPager = findViewById(R.id.pager_main);

        mPagerAdapter = new MainPagerAdapter(this);
        mViewPager.setAdapter(mPagerAdapter);
        mViewPager.setOffscreenPageLimit(mPagerAdapter.getItemCount());
        mViewPager.registerOnPageChangeCallback(new ViewPager2.OnPageChangeCallback() {
            @Override
            public void onPageScrollStateChanged(int state) {
                if (!mSwipeRefresh.isRefreshing()) {
                    mSwipeRefresh.setEnabled(state == ViewPager2.SCROLL_STATE_IDLE);
                }
            }

            @Override
            public void onPageSelected(int position) {
                mTabIndex = position;
                showGameList();
            }
        });

        mMediator = new TabLayoutMediator(mTabLayout, mViewPager, (tab, position) -> {
            switch (position) {
                case PAGE_FRAGMENT_EXTERNAL:
                    tab.setCustomView(getCustomTabView(R.drawable.ic_folder));
                    break;
                case PAGE_FRAGMENT_INSTALLED:
                    tab.setCustomView(getCustomTabView(R.drawable.ic_shop));
                    break;
                case PAGE_FRAGMENT_CONTENT:
                    tab.setCustomView(getCustomTabView(R.drawable.ic_extension));
                    break;
            }
        });
        mMediator.attach();

        mSwipeRefresh = findViewById(R.id.swipe_refresh);
        mSwipeRefresh.setColorSchemeResources(R.color.citra_accent);
        mSwipeRefresh.post(() -> {
            mSwipeRefresh.setDistanceToTriggerSync(mSwipeRefresh.getHeight() / 3);
        });
        mSwipeRefresh.setOnRefreshListener(this::refreshLibrary);

        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
        mTabIndex = pref.getInt(PREF_TAB_INDEX, 0);
        mViewPager.setCurrentItem(mTabIndex, false);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mDirToAdd != null) {
            mTabIndex = 0;
            new RefreshTask(mDirToAdd).execute(false);
            mDirToAdd = null;
        }

        if (mFilesToAdd != null) {
            if (mFilesToAdd.size() > 0) {
                new InstallTask(mFilesToAdd).execute();
            }
            mFilesToAdd = null;
        }

        if (mRefreshPersistedUri) {
            mRefreshPersistedUri = false;
            RefreshPersistedUri(false);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        SharedPreferences.Editor editor = PreferenceManager.getDefaultSharedPreferences(this).edit();
        editor.putInt(PREF_TAB_INDEX, mTabIndex);
        editor.commit();

        sInstance = new WeakReference<>(null);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent result) {
        super.onActivityResult(requestCode, resultCode, result);

        switch (requestCode) {
        case FileBrowserHelper.REQUEST_OPEN_DIRECTORY:
            // If the user picked a file, as opposed to just backing out.
            if (resultCode == RESULT_OK) {
                mDirToAdd = FileBrowserHelper.getSelectedDirectory(result);
            }
            break;
        case FileBrowserHelper.REQUEST_OPEN_FILE:
            if (resultCode == RESULT_OK) {
                String[] files = FileBrowserHelper.getSelectedFiles(result);
                if (files != null) {
                    mFilesToAdd = Arrays.asList(files);
                }
            }
            break;
        case FileBrowserHelper.REQUEST_OPEN_DOCUMENT_TREE:
            if (resultCode == RESULT_OK) {
                Uri uri = result.getData();
                getContentResolver().takePersistableUriPermission(uri,
                        Intent.FLAG_GRANT_READ_URI_PERMISSION |
                                Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
                mRefreshPersistedUri = true;
            }
            break;
        case FileBrowserHelper.REQUEST_OPEN_DOCUMENT:
            if (resultCode == RESULT_OK) {
                Uri uri = result.getData();
                ClipData clipData = result.getClipData();
                mFilesToAdd = new ArrayList<>();
                if (uri != null) {
                    mFilesToAdd.add(uri.toString());
                }
                if (clipData != null) {
                    for (int i = 0; i < clipData.getItemCount(); ++i) {
                        ClipData.Item item = clipData.getItemAt(i);
                        mFilesToAdd.add(item.getUri().toString());
                    }
                }
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
                CitraDirectory.start(this);
                showGameList();
            } else {
                Toast.makeText(this, R.string.write_permission_needed, Toast.LENGTH_SHORT).show();
                refreshExternalGames();
                refreshInstalledGames();
                refreshInstalledContents();
            }
            break;
        default:
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            break;
        }
    }

    private View getCustomTabView(int icon) {
        View view = LayoutInflater.from(this).inflate(R.layout.main_tab_item, null);
        ImageView image = view.findViewById(R.id.tab_icon);
        image.setImageResource(icon);
        return view;
    }

    public void showGameList() {
        if (!CitraDirectory.isInitialized()) {
            // wait for initialized
        } else if (!mIsLoadGameList) {
            mIsLoadGameList = true;
            new LoadGameListTask().execute();
        } else if (mTabIndex == PAGE_FRAGMENT_EXTERNAL) {
            refreshExternalGames();
        } else if (mTabIndex == PAGE_FRAGMENT_INSTALLED) {
            refreshInstalledGames();
        } else if (mTabIndex == PAGE_FRAGMENT_CONTENT) {
            refreshInstalledContents();
        }
    }

    public void refreshLibrary() {
        if (mTabIndex == PAGE_FRAGMENT_EXTERNAL) {
            if (CitraDirectory.isExternalStorageLegacy()) {
                new RefreshTask(false).execute(true);
            } else {
                RefreshPersistedUri(true);
            }
        } else {
            new RefreshTask(true).execute(true);
        }
    }

    private void RefreshPersistedUri(boolean clearCache) {
        List<UriPermission> list = getContentResolver().getPersistedUriPermissions();
        List<Uri> dirs = new ArrayList<>();
        for (UriPermission entity : list) {
            if (entity.isReadPermission()) {
                dirs.add(entity.getUri());
            }
        }
        new RefreshTask(this, dirs).execute(clearCache);
    }

    public void updateProgress(String name, long written, long total) {
        if (written < total) {
            mProgressBar.setVisibility(View.VISIBLE);
            mProgressText.setVisibility(View.VISIBLE);
            mProgressText.setText((written * 100 / total) + "%");
        } else {
            mProgressBar.setVisibility(View.INVISIBLE);
            mProgressText.setVisibility(View.INVISIBLE);
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            if (total == 0) {
                if (written == 0) {
                    Toast.makeText(this, getString(R.string.cia_install_success), Toast.LENGTH_LONG).show();
                } else {
                    Toast.makeText(this, "Error: " + name, Toast.LENGTH_LONG).show();
                }
            }
        }
    }

    public void addNetPlayMessage(String msg) {
        if (msg.isEmpty()) {
            return;
        }
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }

    private void refreshExternalGames() {
        setGames(PAGE_FRAGMENT_EXTERNAL, mExternalGames);
    }

    private void refreshInstalledGames() {
        setGames(PAGE_FRAGMENT_INSTALLED, mInstalledGames);
    }

    private void refreshInstalledContents() {
        setGames(PAGE_FRAGMENT_CONTENT, mInstalledContents);
    }

    private void setGames(int position, List<GameFile> games) {
        Fragment fragment = getSupportFragmentManager().findFragmentByTag("f" + position);
        if (fragment != null) {
            ((MainPageFragment)fragment).setGames(games);
        } else if (!mRunningHandlers[position]) {
            mRunningHandlers[position] = true;
            mHandler.postDelayed(() -> {
                mRunningHandlers[position] = false;
                setGames(position, games);
            }, 60);
        }
    }

    private static class MainPagerAdapter extends FragmentStateAdapter {

        public MainPagerAdapter(@NonNull FragmentActivity fragmentActivity) {
            super(fragmentActivity);
        }

        @NonNull
        @Override
        public Fragment createFragment(int position) {
            return MainPageFragment.newInstance(position);
        }

        @Override
        public int getItemCount() {
            return 3;
        }
    }
}
