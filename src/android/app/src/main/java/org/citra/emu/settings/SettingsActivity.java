package org.citra.emu.settings;

import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.fragment.app.FragmentTransaction;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.BooleanSetting;
import org.citra.emu.settings.model.IntSetting;
import org.citra.emu.settings.model.StringSetting;
import org.citra.emu.utils.CitraDirectory;
import org.citra.emu.utils.EsDeFrontendRegistration;
import org.citra.emu.utils.FileBrowserHelper;
import org.citra.emu.utils.PermissionsHandler;

import java.io.File;

public final class SettingsActivity extends AppCompatActivity {
    private static final String FRAGMENT_TAG = "settings";
    private static final int REQUEST_CODE_STATES_DIRECTORY =
        FileBrowserHelper.REQUEST_OPEN_DIRECTORY;
    private static final int REQUEST_CODE_ES_DE_DIRECTORY =
        FileBrowserHelper.REQUEST_OPEN_DIRECTORY_ES_DE;

    private static final String KEY_SHOULD_SAVE = "should_save";
    private static final String KEY_MENU_TAG = "menu_tag";
    private static final String KEY_GAME_ID = "game_id";
    private static final String KEY_GAME_NAME = "game_name";
    private static final String KEY_PENDING_DIRECTORY_SETTING = "pending_directory_setting";
    private static final String KEY_PENDING_ES_DE_REGISTRATION = "pending_es_de_registration";

    private Settings mSettings = new Settings();
    private boolean mShouldSave;
    private boolean mPendingEsDeRegistration;
    private MenuTag mMenuTag;
    private String mGameId;
    private String mGameName;
    private String mPendingDirectorySettingKey;

    public static void launch(Context context, MenuTag menuTag, String gameId, String gameName) {
        Intent settings = new Intent(context, SettingsActivity.class);
        settings.putExtra(KEY_MENU_TAG, menuTag.toString());
        settings.putExtra(KEY_GAME_ID, gameId);
        settings.putExtra(KEY_GAME_NAME, gameName);
        context.startActivity(settings);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        if (savedInstanceState == null) {
            Intent intent = getIntent();
            String menuTagStr = intent.getStringExtra(KEY_MENU_TAG);
            mMenuTag = MenuTag.getMenuTag(menuTagStr);
            mGameId = intent.getStringExtra(KEY_GAME_ID);
            mGameName = intent.getStringExtra(KEY_GAME_NAME);
        } else {
            String menuTagStr = savedInstanceState.getString(KEY_MENU_TAG);
            mShouldSave = savedInstanceState.getBoolean(KEY_SHOULD_SAVE);
            mMenuTag = MenuTag.getMenuTag(menuTagStr);
            mGameId = savedInstanceState.getString(KEY_GAME_ID);
            mGameName = savedInstanceState.getString(KEY_GAME_NAME);
            mPendingDirectorySettingKey =
                savedInstanceState.getString(KEY_PENDING_DIRECTORY_SETTING);
            mPendingEsDeRegistration =
                savedInstanceState.getBoolean(KEY_PENDING_ES_DE_REGISTRATION);
        }

        if (!mGameName.isEmpty()) {
            setTitle(mGameName);
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        // Critical: If super method is not called, rotations will be busted.
        super.onSaveInstanceState(outState);

        outState.putBoolean(KEY_SHOULD_SAVE, mShouldSave);
        outState.putString(KEY_MENU_TAG, mMenuTag.toString());
        outState.putString(KEY_GAME_ID, mGameId);
        outState.putString(KEY_GAME_NAME, mGameName);
        outState.putString(KEY_PENDING_DIRECTORY_SETTING, mPendingDirectorySettingKey);
        outState.putBoolean(KEY_PENDING_ES_DE_REGISTRATION, mPendingEsDeRegistration);
    }

    @Override
    protected void onStart() {
        super.onStart();

        if (mSettings.isEmpty()) {
            mSettings.loadSettings(mGameId);
            showSettingsFragment(mMenuTag, null, false, mGameId);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (requestCode == REQUEST_CODE_STATES_DIRECTORY && resultCode == RESULT_OK && data != null) {
            String path = FileBrowserHelper.getSelectedDirectory(data);
            if (path != null && mPendingDirectorySettingKey != null) {
                setCustomStoragePath(mPendingDirectorySettingKey, path);
            }
            mPendingDirectorySettingKey = null;
        } else if (requestCode == REQUEST_CODE_ES_DE_DIRECTORY) {
            if (resultCode == RESULT_OK && data != null) {
                String path = FileBrowserHelper.getSelectedDirectory(data);
                if (path != null) {
                    handleEsDeCustomSystemsPath(path);
                }
            } else {
                mPendingEsDeRegistration = false;
                mPendingDirectorySettingKey = null;
                refreshSettingsList();
            }
        } else if (requestCode == FileBrowserHelper.REQUEST_OPEN_DOCUMENT_TREE_ES_DE) {
            if (resultCode == RESULT_OK && data != null) {
                handleEsDeCustomSystemsResult(data);
            } else {
                mPendingEsDeRegistration = false;
                refreshSettingsList();
            }
        }
    }

    @Override
    protected void onStop() {
        super.onStop();

        if (mSettings != null && isFinishing() && mShouldSave) {
            mSettings.saveSettings();
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_setting, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == R.id.menu_reset_setting) {
            File ini = new File(CitraDirectory.getConfigFile());
            File ini2 = new File(CitraDirectory.getConfigDirectory() + "/config-games.ini");
            try {
                ini.delete();
                ini2.delete();
            } catch (Exception e) {
                // ignore
            }
            CitraDirectory.setStatesDirectoryOverride("");
            CitraDirectory.setSDMCDirectoryOverride("");
            mSettings.loadSettings(mGameId);
            // show settings
            SettingsFragment fragment = getSettingsFragment();
            if (fragment != null) {
                fragment.showSettingsList(mSettings);
            }
            Toast.makeText(this, R.string.reset_setting, Toast.LENGTH_SHORT).show();
            return true;
        } else if (item.getItemId() == R.id.menu_reset_cache) {
            if (CitraDirectory.deleteAllFiles(CitraDirectory.getCacheDirectory())) {
                Toast.makeText(this, R.string.delete_success, Toast.LENGTH_SHORT).show();
            }
        }
        return false;
    }

    public void showSettingsFragment(MenuTag menuTag, Bundle extras, boolean addToStack,
                                     String gameID) {
        FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

        if (addToStack) {
            if (areSystemAnimationsEnabled()) {
                transaction.setCustomAnimations(R.animator.settings_enter, R.animator.settings_exit,
                                                R.animator.settings_pop_enter,
                                                R.animator.setttings_pop_exit);
            }

            transaction.addToBackStack(null);
        }
        transaction.replace(R.id.frame_content,
                            SettingsFragment.newInstance(menuTag, gameID, extras), FRAGMENT_TAG);
        transaction.commit();

        // show settings
        SettingsFragment fragment = getSettingsFragment();
        if (fragment != null) {
            fragment.showSettingsList(mSettings);
        }
    }

    private boolean areSystemAnimationsEnabled() {
        float duration = android.provider.Settings.Global.getFloat(
            getContentResolver(), android.provider.Settings.Global.ANIMATOR_DURATION_SCALE, 1);

        float transition = android.provider.Settings.Global.getFloat(
            getContentResolver(), android.provider.Settings.Global.TRANSITION_ANIMATION_SCALE, 1);

        return duration != 0 && transition != 0;
    }

    private SettingsFragment getSettingsFragment() {
        return (SettingsFragment)getSupportFragmentManager().findFragmentByTag(FRAGMENT_TAG);
    }

    public Settings getSettings() {
        return mSettings;
    }

    public void setSettings(Settings settings) {
        mSettings = settings;
    }

    public void putSetting(Setting setting) {
        mSettings.getSection(setting.getSection()).putSetting(setting);
    }

    public void loadSubMenu(MenuTag menuKey) {
        showSettingsFragment(menuKey, null, true, mGameId);
    }

    public void setSettingChanged() {
        mShouldSave = true;
    }

    public void refreshSettingsList() {
        SettingsFragment fragment = getSettingsFragment();
        if (fragment != null) {
            fragment.showSettingsList(mSettings);
        }
    }

    public void applyLargeScreenTopAutoFit() {
        int[] size = getCurrentDisplaySize();
        int portraitWidth = Math.min(size[0], size[1]);
        int portraitHeight = Math.max(size[0], size[1]);
        int landscapeWidth = portraitHeight;
        int landscapeHeight = portraitWidth;

        int marginLeft = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                            SettingsFile.KEY_LAYOUT_MARGIN_LEFT, 0);
        int marginTop = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                           SettingsFile.KEY_LAYOUT_MARGIN_TOP, 0);
        int marginRight = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                             SettingsFile.KEY_LAYOUT_MARGIN_RIGHT, 0);
        int marginBottom = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                              SettingsFile.KEY_LAYOUT_MARGIN_BOTTOM, 0);
        int landscapeMarginLeft = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                                     SettingsFile.KEY_LANDSCAPE_LAYOUT_MARGIN_LEFT,
                                                     marginLeft);
        int landscapeMarginTop = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                                    SettingsFile.KEY_LANDSCAPE_LAYOUT_MARGIN_TOP,
                                                    marginTop);
        int landscapeMarginRight = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                                      SettingsFile.KEY_LANDSCAPE_LAYOUT_MARGIN_RIGHT,
                                                      marginRight);
        int landscapeMarginBottom = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                                       SettingsFile.KEY_LANDSCAPE_LAYOUT_MARGIN_BOTTOM,
                                                       marginBottom);
        boolean portraitSwap = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                                  SettingsFile.KEY_SWAP_SCREEN, 0) > 0;
        boolean landscapeSwap = getIntSettingValue(Settings.SECTION_INI_RENDERER,
                                                   SettingsFile.KEY_LANDSCAPE_SWAP_SCREEN,
                                                   portraitSwap ? 1 : 0) > 0;

        int portraitAuto = NativeLibrary.getLargeScreenTopAutoFitProportionForDimensions(
            portraitWidth, portraitHeight, marginLeft, marginTop, marginRight, marginBottom,
            portraitSwap);
        int landscapeAuto = NativeLibrary.getLargeScreenTopAutoFitProportionForDimensions(
            landscapeWidth, landscapeHeight, landscapeMarginLeft, landscapeMarginTop,
            landscapeMarginRight, landscapeMarginBottom, landscapeSwap);

        putSetting(new IntSetting(SettingsFile.KEY_LARGE_SCREEN_PROPORTION,
                                  Settings.SECTION_INI_RENDERER, portraitAuto));
        putSetting(new IntSetting(SettingsFile.KEY_LANDSCAPE_LARGE_SCREEN_PROPORTION,
                                  Settings.SECTION_INI_RENDERER, landscapeAuto));
        setSettingChanged();
        refreshSettingsList();
    }

    public void handleScreenLayoutSettingChanged(String key) {
        if (key == null) {
            return;
        }

        if (SettingsFile.KEY_LAYOUT_OPTION.equals(key)) {
            refreshSettingsList();
            if (isLargeScreenTopAutoFitEnabled()) {
                applyLargeScreenTopAutoFit();
            }
            return;
        }

        if (SettingsFile.KEY_LARGE_SCREEN_AUTO_FIT.equals(key)) {
            if (isLargeScreenTopAutoFitEnabled()) {
                applyLargeScreenTopAutoFit();
            } else {
                refreshSettingsList();
            }
            return;
        }

        if (isLargeScreenTopAutoFitEnabled() && isLargeScreenTopAutoFitRelevantKey(key)) {
            applyLargeScreenTopAutoFit();
        }
    }

    public void openStoragePathPicker(String settingKey) {
        if (EsDeFrontendRegistration.KEY_ES_DE_CUSTOM_SYSTEMS_PATH.equals(settingKey)) {
            mPendingDirectorySettingKey = settingKey;
            FileBrowserHelper.openDirectoryPicker(
                this, REQUEST_CODE_ES_DE_DIRECTORY,
                EsDeFrontendRegistration.getDefaultFolderSummary());
            return;
        }

        mPendingDirectorySettingKey = settingKey;
        FileBrowserHelper.openDirectoryPicker(this);
    }

    public void setCustomStoragePath(String settingKey, String path) {
        if (EsDeFrontendRegistration.KEY_ES_DE_CUSTOM_SYSTEMS_PATH.equals(settingKey)) {
            EsDeFrontendRegistration.saveCustomSystemsPath(this, path);
            refreshSettingsList();
            return;
        }

        StringSetting setting =
            new StringSetting(settingKey, Settings.SECTION_INI_CORE, path);
        putSetting(setting);
        if (SettingsFile.KEY_STATES_PATH.equals(settingKey)) {
            CitraDirectory.setStatesDirectoryOverride(path);
        } else if (SettingsFile.KEY_SDMC_PATH.equals(settingKey)) {
            CitraDirectory.setSDMCDirectoryOverride(path);
        }
        mShouldSave = true;

        SettingsFragment fragment = getSettingsFragment();
        if (fragment != null) {
            fragment.showSettingsList(mSettings);
        }
    }

    public void registerEsDeFrontend() {
        String savedPath = EsDeFrontendRegistration.getSavedCustomSystemsPath(this);
        if (savedPath != null && EsDeFrontendRegistration.canRegisterUsingDefaultPath(this)) {
            try {
                EsDeFrontendRegistration.registerUsingSelectedPath(this, savedPath);
                Toast.makeText(this, R.string.es_de_registration_success, Toast.LENGTH_SHORT).show();
                refreshSettingsList();
                return;
            } catch (Exception e) {
                Log.w("citra", "Saved ES-DE folder path could not be reused, falling back",
                      e);
                EsDeFrontendRegistration.clearSavedCustomSystemsPath(this);
                refreshSettingsList();
            }
        }

        if (EsDeFrontendRegistration.canRegisterUsingDefaultPath(this)) {
            try {
                EsDeFrontendRegistration.registerUsingDefaultPath(this);
                Toast.makeText(this, R.string.es_de_registration_success, Toast.LENGTH_SHORT).show();
                refreshSettingsList();
                return;
            } catch (Exception e) {
                Log.w("citra", "Default ES-DE path registration failed, trying persisted SAF path",
                      e);
            }
        } else if (!PermissionsHandler.checkWritePermission(this)) {
            return;
        }

        Uri savedUri = EsDeFrontendRegistration.getSavedCustomSystemsUri(this);
        if (savedUri != null) {
            try {
                EsDeFrontendRegistration.register(this, savedUri);
                Toast.makeText(this, R.string.es_de_registration_success, Toast.LENGTH_SHORT).show();
                refreshSettingsList();
                return;
            } catch (Exception e) {
                Log.w("citra", "Saved ES-DE folder could not be reused, asking user to pick it again",
                      e);
                EsDeFrontendRegistration.clearSavedCustomSystemsUri(this);
                refreshSettingsList();
            }
        }

        mPendingEsDeRegistration = true;
        FileBrowserHelper.openEsDeCustomSystemsTree(this,
                                                    FileBrowserHelper.REQUEST_OPEN_DOCUMENT_TREE_ES_DE);
    }

    public void selectEsDeCustomSystemsFolder() {
        mPendingEsDeRegistration = false;
        if (EsDeFrontendRegistration.canRegisterUsingDefaultPath(this)) {
            FileBrowserHelper.openDirectoryPicker(
                this, REQUEST_CODE_ES_DE_DIRECTORY,
                EsDeFrontendRegistration.getDefaultFolderSummary());
            return;
        }

        if (!PermissionsHandler.checkWritePermission(this)) {
            return;
        }

        FileBrowserHelper.openDirectoryPicker(
            this, REQUEST_CODE_ES_DE_DIRECTORY,
            EsDeFrontendRegistration.getDefaultFolderSummary());
    }

    private void handleEsDeCustomSystemsResult(Intent data) {
        Uri uri = data.getData();
        if (uri == null) {
            return;
        }

        final int takeFlags = (data.getFlags() &
                               (Intent.FLAG_GRANT_READ_URI_PERMISSION |
                                Intent.FLAG_GRANT_WRITE_URI_PERMISSION)) |
                              Intent.FLAG_GRANT_READ_URI_PERMISSION |
                              Intent.FLAG_GRANT_WRITE_URI_PERMISSION;

        try {
            getContentResolver().takePersistableUriPermission(uri, takeFlags);
            EsDeFrontendRegistration.saveCustomSystemsUri(this, uri);

            if (mPendingEsDeRegistration) {
                EsDeFrontendRegistration.register(this, uri);
                Toast.makeText(this, R.string.es_de_registration_success, Toast.LENGTH_SHORT)
                    .show();
            } else {
                Toast.makeText(this, R.string.es_de_folder_saved, Toast.LENGTH_SHORT).show();
            }
        } catch (IllegalArgumentException e) {
            Log.w("citra", "Invalid ES-DE folder selected: " + uri, e);
            Toast.makeText(this, R.string.es_de_select_custom_systems_folder, Toast.LENGTH_LONG)
                .show();
            EsDeFrontendRegistration.clearSavedCustomSystemsUri(this);
        } catch (Exception e) {
            Log.e("citra", "Failed to update ES-DE registration", e);
            Toast.makeText(this, R.string.es_de_registration_failed, Toast.LENGTH_LONG).show();
            EsDeFrontendRegistration.clearSavedCustomSystemsUri(this);
        } finally {
            mPendingEsDeRegistration = false;
            mPendingDirectorySettingKey = null;
            refreshSettingsList();
        }
    }

    private void handleEsDeCustomSystemsPath(String path) {
        try {
            EsDeFrontendRegistration.saveCustomSystemsPath(this, path);

            if (mPendingEsDeRegistration) {
                EsDeFrontendRegistration.registerUsingSelectedPath(this, path);
                Toast.makeText(this, R.string.es_de_registration_success, Toast.LENGTH_SHORT)
                    .show();
            } else {
                Toast.makeText(this, R.string.es_de_folder_saved, Toast.LENGTH_SHORT).show();
            }
        } catch (IllegalArgumentException e) {
            Log.w("citra", "Invalid ES-DE folder selected: " + path, e);
            Toast.makeText(this, R.string.es_de_select_custom_systems_folder, Toast.LENGTH_LONG)
                .show();
            EsDeFrontendRegistration.clearSavedCustomSystemsPath(this);
        } catch (Exception e) {
            Log.e("citra", "Failed to update ES-DE registration", e);
            Toast.makeText(this, R.string.es_de_registration_failed, Toast.LENGTH_LONG).show();
            EsDeFrontendRegistration.clearSavedCustomSystemsPath(this);
        } finally {
            mPendingEsDeRegistration = false;
            refreshSettingsList();
        }
    }

    private int[] getCurrentDisplaySize() {
        DisplayMetrics metrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getRealMetrics(metrics);
        return new int[]{metrics.widthPixels, metrics.heightPixels};
    }

    private boolean isLargeScreenTopAutoFitEnabled() {
        final int layoutOption =
            normalizeLayoutOption(
                getIntSettingValue(Settings.SECTION_INI_RENDERER, SettingsFile.KEY_LAYOUT_OPTION, 0));
        if (layoutOption != 2) {
            return false;
        }
        return getBooleanSettingValue(Settings.SECTION_INI_RENDERER,
                                      SettingsFile.KEY_LARGE_SCREEN_AUTO_FIT, false);
    }

    private boolean isLargeScreenTopAutoFitRelevantKey(String key) {
        return SettingsFile.KEY_SWAP_SCREEN.equals(key) ||
               SettingsFile.KEY_LARGE_SCREEN_SECONDARY_LEFT.equals(key) ||
               SettingsFile.KEY_LARGE_SCREEN_SECONDARY_TOP.equals(key) ||
               SettingsFile.KEY_LAYOUT_MARGIN_LEFT.equals(key) ||
               SettingsFile.KEY_LAYOUT_MARGIN_TOP.equals(key) ||
               SettingsFile.KEY_LAYOUT_MARGIN_RIGHT.equals(key) ||
               SettingsFile.KEY_LAYOUT_MARGIN_BOTTOM.equals(key);
    }

    private int normalizeLayoutOption(int layoutOption) {
        return layoutOption == 4 ? 2 : layoutOption;
    }

    private int getIntSettingValue(String sectionName, String key, int defaultValue) {
        Setting setting = mSettings.getSection(sectionName).getSetting(key);
        if (setting instanceof IntSetting) {
            return ((IntSetting)setting).getValue();
        }
        return defaultValue;
    }

    private boolean getBooleanSettingValue(String sectionName, String key, boolean defaultValue) {
        Setting setting = mSettings.getSection(sectionName).getSetting(key);
        if (setting instanceof BooleanSetting) {
            return ((BooleanSetting)setting).getValue();
        }
        return defaultValue;
    }
}
