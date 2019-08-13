package org.citra.emu.ui;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

import java.lang.ref.WeakReference;

public final class EmulationActivity extends AppCompatActivity {
    private static final String EXTRA_GAMEPATH = "SelectedGames";
    private static WeakReference<EmulationActivity> sInstance = new WeakReference<>(null);
    private String mPath;
    private View mDecorView;
    private boolean mMenuVisible;
    private boolean mStopEmulation;
    private EmulationFragment mEmulationFragment;

    public static void launch(Context context, String path) {
        Intent intent = new Intent(context, EmulationActivity.class);
        intent.putExtra(EXTRA_GAMEPATH, path);
        context.startActivity(intent);
    }

    public static EmulationActivity get() {
        return sInstance.get();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_emulation);
        sInstance = new WeakReference<>(this);

        if (savedInstanceState == null) {
            Intent intent = getIntent();
            mPath = intent.getStringExtra(EXTRA_GAMEPATH);
        } else {
            mPath = savedInstanceState.getString(EXTRA_GAMEPATH);
        }

        // Get a handle to the Window containing the UI.
        mDecorView = getWindow().getDecorView();
        mDecorView.setOnSystemUiVisibilityChangeListener(visibility ->
        {
            if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                // Go back to immersive fullscreen mode in 3s
                Handler handler = new Handler(getMainLooper());
                handler.postDelayed(this::enableFullscreenImmersive, 3000 /* 3s */);
            }
        });
        mStopEmulation = false;

        // Find or create the EmulationFragment
        mEmulationFragment = (EmulationFragment) getSupportFragmentManager()
                .findFragmentById(R.id.fragment_emulation);
        if (mEmulationFragment == null) {
            mEmulationFragment = EmulationFragment.newInstance(mPath);
            getSupportFragmentManager().beginTransaction()
                    .add(R.id.fragment_emulation, mEmulationFragment)
                    .commit();
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        outState.putString(EXTRA_GAMEPATH, mPath);
        super.onSaveInstanceState(outState);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_emulation, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_emulation_screenshot:
                NativeLibrary.SaveScreenShot();
                return true;

            case R.id.menu_running_setting:
                RunningSettingDialog.newInstance()
                        .show(getSupportFragmentManager(), "RunningSettingDialog");
                return true;

            case R.id.menu_emulation_edit_layout:
                mEmulationFragment.startConfiguringControls();
                return true;
        }

        return false;
    }

    private void enableFullscreenImmersive() {
        if (mStopEmulation) {
            return;
        }
        mMenuVisible = false;
        // It would be nice to use IMMERSIVE_STICKY, but that doesn't show the toolbar.
        mDecorView.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
                        View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
                        View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
                        View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
                        View.SYSTEM_UI_FLAG_FULLSCREEN |
                        View.SYSTEM_UI_FLAG_IMMERSIVE);
    }

    private void disableFullscreenImmersive() {
        mMenuVisible = true;
        mDecorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
    }

    @Override
    public void onBackPressed() {
        if (mMenuVisible) {
            mStopEmulation = true;
            mEmulationFragment.stopEmulation();
            finish();
        } else {
            disableFullscreenImmersive();
        }
    }
}
