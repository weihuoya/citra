package org.citra.emu.settings;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.FragmentTransaction;
import android.support.v7.app.AppCompatActivity;
import org.citra.emu.R;
import org.citra.emu.settings.model.Setting;

public final class SettingsActivity extends AppCompatActivity {

    private static final String ARG_MENU_TAG = "menu_tag";
    private static final String ARG_GAME_ID = "game_id";
    private static final String FRAGMENT_TAG = "settings";

    private static final String KEY_SHOULD_SAVE = "should_save";
    private static final String KEY_MENU_TAG = "menu_tag";
    private static final String KEY_GAME_ID = "game_id";

    private Settings mSettings = new Settings();
    private int mStackCount;
    private boolean mShouldSave;
    private MenuTag mMenuTag;
    private String mGameId;

    public static void launch(Context context, MenuTag menuTag, String gameId) {
        Intent settings = new Intent(context, SettingsActivity.class);
        settings.putExtra(ARG_MENU_TAG, menuTag);
        settings.putExtra(ARG_GAME_ID, gameId);
        context.startActivity(settings);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        if (savedInstanceState == null) {
            Intent intent = getIntent();
            mMenuTag = (MenuTag)intent.getSerializableExtra(ARG_MENU_TAG);
            mGameId = intent.getStringExtra(ARG_GAME_ID);
        } else {
            String menuTagStr = savedInstanceState.getString(KEY_MENU_TAG);
            mShouldSave = savedInstanceState.getBoolean(KEY_SHOULD_SAVE);
            mMenuTag = MenuTag.getMenuTag(menuTagStr);
            mGameId = savedInstanceState.getString(KEY_GAME_ID);
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        // Critical: If super method is not called, rotations will be busted.
        super.onSaveInstanceState(outState);

        outState.putBoolean(KEY_SHOULD_SAVE, mShouldSave);
        outState.putString(KEY_MENU_TAG, mMenuTag.toString());
        outState.putString(KEY_GAME_ID, mGameId);
    }

    @Override
    protected void onStart() {
        super.onStart();

        if (mSettings.isEmpty()) {
            mSettings.loadSettings(mGameId);
        }
        showSettingsFragment(mMenuTag, null, false, mGameId);
    }

    @Override
    protected void onStop() {
        super.onStop();

        if (mSettings != null && isFinishing() && mShouldSave) {
            mSettings.saveSettings();
        }
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
            mStackCount++;
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
}
