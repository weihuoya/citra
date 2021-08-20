package org.citra.emu.ui;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.view.DisplayCutout;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.io.InputStream;
import java.lang.ref.WeakReference;
import java.util.List;
import java.util.Map;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.settings.Settings;
import org.citra.emu.settings.SettingsFile;
import org.citra.emu.settings.model.Setting;
import org.citra.emu.settings.model.SettingSection;
import org.citra.emu.utils.ControllerMappingHelper;
import org.citra.emu.utils.DirectoryInitialization;

public final class EmulationActivity extends AppCompatActivity {
    public static final String EXTRA_GAME_ID = "GameId";
    public static final String EXTRA_GAME_NAME = "GameName";
    public static final String EXTRA_GAME_PATH = "GamePath";
    public static final int REQUEST_PICK_IMAGE = 1;
    public static final int REQUEST_CHEAT_CODE = 2;

    private static WeakReference<EmulationActivity> sInstance = new WeakReference<>(null);

    @SuppressLint("StaticFieldLeak")
    private class InitTask extends AsyncTask<Context, Void, Map<Integer, Bitmap>> {
        @Override
        protected Map<Integer, Bitmap> doInBackground(Context... contexts) {
            if (!DirectoryInitialization.isInitialized()) {
                DirectoryInitialization.start(contexts[0]);
            }
            Settings settings = new Settings();
            settings.loadSettings(mGameId);
            SettingSection section = settings.getSection(Settings.SECTION_INI_CORE);
            Setting setting = section.getSetting(SettingsFile.KEY_THEME_PACKAGE);
            String theme = setting != null ? setting.getValueAsString() : "";
            return DirectoryInitialization.loadInputOverlay(contexts[0], theme);
        }

        @Override
        protected void onPostExecute(Map<Integer, Bitmap> result) {
            mBitmaps = result;
            refreshControls();
            updateBackgroundImage();
        }
    }

    private String mGameId;
    private String mGameName;
    private String mGamePath;
    private boolean mMenuVisible;
    private EmulationFragment mEmulationFragment;
    private Map<Integer, Bitmap> mBitmaps;

    private int mSafeInsetLeft = 0;
    private int mSafeInsetTop = 0;
    private int mSafeInsetRight = 0;
    private int mSafeInsetBottom = 0;
    private int mDisplayRotation = 0;
    private float mScaledDensity = 1.0f;

    public static void launch(Context context, GameFile game) {
        Intent intent = new Intent(context, EmulationActivity.class);
        intent.putExtra(EXTRA_GAME_ID, game.getId());
        intent.putExtra(EXTRA_GAME_NAME, game.getName());
        intent.putExtra(EXTRA_GAME_PATH, game.getPath());
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
            mGameId = intent.getStringExtra(EXTRA_GAME_ID);
            mGameName = intent.getStringExtra(EXTRA_GAME_NAME);
            mGamePath = intent.getStringExtra(EXTRA_GAME_PATH);
        } else {
            mGameId = savedInstanceState.getString(EXTRA_GAME_ID);
            mGameName = savedInstanceState.getString(EXTRA_GAME_NAME);
            mGamePath = savedInstanceState.getString(EXTRA_GAME_PATH);
        }

        // Find or create the EmulationFragment
        mEmulationFragment = (EmulationFragment)getSupportFragmentManager().findFragmentById(
            R.id.fragment_emulation);
        if (mEmulationFragment == null) {
            mEmulationFragment = EmulationFragment.newInstance(mGamePath);
            getSupportFragmentManager()
                .beginTransaction()
                .add(R.id.fragment_emulation, mEmulationFragment)
                .commit();
        }

        setTitle(mGameName);
        new InitTask().execute(this);
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        outState.putString(EXTRA_GAME_ID, mGameId);
        outState.putString(EXTRA_GAME_NAME, mGameName);
        outState.putString(EXTRA_GAME_PATH, mGamePath);
        super.onSaveInstanceState(outState);
    }

    @Override
    public void onBackPressed() {
        if (mMenuVisible) {
            mEmulationFragment.stopEmulation();
            finish();
            sInstance = new WeakReference<>(null);
        } else {
            mMenuVisible = true;
            RunningSettingDialog dialog = RunningSettingDialog.newInstance();
            dialog.show(getSupportFragmentManager(), "RunningSettingDialog");
            dialog.setOnDismissListener(v -> {
                mMenuVisible = false;
                hideSystemUI();
            });
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        hideSystemUI();
    }

    private void hideSystemUI() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    @Override
    public void onAttachedToWindow() {
        super.onAttachedToWindow();
        updateDisplayConfig();
    }

    // Gets button presses
    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        if (mMenuVisible) {
            return super.dispatchKeyEvent(event);
        }

        InputDevice input = event.getDevice();
        int button = event.getKeyCode();
        int action;
        switch (event.getAction()) {
        case KeyEvent.ACTION_DOWN:
            action = NativeLibrary.ButtonState.PRESSED;
            break;
        case KeyEvent.ACTION_UP:
            action = NativeLibrary.ButtonState.RELEASED;
            break;
        default:
            return super.dispatchKeyEvent(event);
        }

        if (input != null && NativeLibrary.KeyEvent(button, action)) {
            return true;
        } else
            return super.dispatchKeyEvent(event);
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event) {
        if (mMenuVisible) {
            return super.dispatchGenericMotionEvent(event);
        }

        if (((event.getSource() & InputDevice.SOURCE_CLASS_JOYSTICK) == 0)) {
            return super.dispatchGenericMotionEvent(event);
        }

        // Don't attempt to do anything if we are disconnecting a device.
        if (event.getActionMasked() == MotionEvent.ACTION_CANCEL)
            return true;

        InputDevice input = event.getDevice();
        List<InputDevice.MotionRange> motions = input.getMotionRanges();

        for (InputDevice.MotionRange range : motions) {
            int axis = range.getAxis();
            float origValue = event.getAxisValue(axis);
            float value = ControllerMappingHelper.scaleAxis(input, axis, origValue);
            // If the input is still in the "flat" area, that means it's really zero.
            // This is used to compensate for imprecision in joysticks.
            if (Math.abs(value) > range.getFlat()) {
                NativeLibrary.MoveEvent(axis, value);
            } else {
                NativeLibrary.MoveEvent(axis, 0.0f);
            }
        }

        return true;
    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (requestCode == REQUEST_PICK_IMAGE) {
            if (data == null || data.getData() == null || resultCode != Activity.RESULT_OK) {
                NativeLibrary.HandleImage(null, 0, 0);
                return;
            }

            InputStream inputStream;
            try {
                inputStream = getContentResolver().openInputStream(data.getData());
            } catch (IOException e) {
                NativeLibrary.HandleImage(null, 0, 0);
                return;
            }

            Bitmap image = BitmapFactory.decodeStream(inputStream);
            int srcWidth = image.getWidth();
            int srcHeight = image.getHeight();
            int dstWidth = getIntent().getIntExtra("width", srcWidth);
            int dstHeight = getIntent().getIntExtra("height", srcHeight);
            int[] pixels = new int[dstWidth * dstHeight];
            image = adjustImage(image, dstWidth, dstHeight);
            image.getPixels(pixels, 0, dstWidth, 0, 0, dstWidth, dstHeight);
            NativeLibrary.HandleImage(pixels, dstWidth, dstHeight);
        } else if (requestCode == REQUEST_CHEAT_CODE) {
            NativeLibrary.reloadCheatCode();
        }
    }

    public void updateDisplayConfig() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            DisplayCutout cutout = getWindow().getDecorView().getRootWindowInsets().getDisplayCutout();
            if (cutout != null) {
                mSafeInsetLeft = cutout.getSafeInsetLeft();
                mSafeInsetTop = cutout.getSafeInsetTop();
                mSafeInsetRight = cutout.getSafeInsetRight();
                mSafeInsetBottom = cutout.getSafeInsetBottom();
            }
        }

        mDisplayRotation = getWindowManager().getDefaultDisplay().getRotation();
        mScaledDensity = getResources().getDisplayMetrics().scaledDensity;
    }

    private Bitmap adjustImage(Bitmap image, int width, int height) {
        int srcWidth = image.getWidth();
        int srcHeight = image.getHeight();
        if (width == srcWidth && height == srcHeight) {
            return image;
        }

        int cropX;
        int cropY;
        int cropWidth;
        int cropHeight;

        float ratioNeed = width / (float)height;
        float ratioHave = srcWidth / (float)srcHeight;
        if (ratioNeed > ratioHave) {
            cropWidth = srcWidth;
            cropHeight = (int)(srcWidth / ratioNeed);
            cropX = 0;
            cropY = (srcHeight - cropHeight) / 2;
        } else {
            cropWidth = (int)(srcHeight * ratioNeed);
            cropHeight = srcHeight;
            cropX = (srcWidth - cropWidth) / 2;
            cropY = 0;
        }

        Bitmap cropped = Bitmap.createBitmap(image, cropX, cropY, cropWidth, cropHeight);
        if (cropWidth == width && cropHeight == height) {
            return cropped;
        } else {
            Matrix matrix = new Matrix();
            matrix.postScale(width / (float)cropWidth, height / (float)cropHeight);
            return Bitmap.createBitmap(cropped, 0, 0, cropWidth, cropHeight, matrix, true);
        }
    }

    public void pickImage(int width, int height) {
        Intent intent = getIntent() != null ? getIntent() : new Intent();
        intent.putExtra("width", width);
        intent.putExtra("height", height);
        setIntent(intent);
        //
        intent = new Intent();
        intent.setType("image/*");
        intent.setAction(Intent.ACTION_GET_CONTENT);
        startActivityForResult(Intent.createChooser(intent, "Select Picture"), REQUEST_PICK_IMAGE);
    }

    public void showInputBoxDialog(int maxLength, String error, String hint, String button0, String button1,
                                   String button2) {
        KeyboardDialog.newInstance(maxLength, error, hint, button0, button1, button2)
            .show(getSupportFragmentManager(), "KeyboardDialog");
    }

    public void showMiiSelectorDialog(boolean cancel, String title, String[] miis) {
        MiiSelectorDialog.newInstance(cancel, title, miis)
            .show(getSupportFragmentManager(), "MiiSelectorDialog");
    }

    private AmiiboDialog mAmiiboDialog;
    public void handleNFCScanning(boolean isScanning) {
        if (isScanning) {
            if (mAmiiboDialog == null) {
                mAmiiboDialog = AmiiboDialog.newInstance();
                mAmiiboDialog.show(getSupportFragmentManager(), "AmiiboDialog");
            }
        } else if (mAmiiboDialog != null) {
            mAmiiboDialog.dismiss();
            mAmiiboDialog = null;
        }
    }

    public String getGameName() {
        return mGameName;
    }

    public void refreshControls() {
        mEmulationFragment.refreshControls();
    }

    public void startConfiguringControls() {
        mEmulationFragment.startConfiguringControls();
    }

    public void startConfiguringLayout() {
        mEmulationFragment.startConfiguringLayout();
    }

    public void setLassoOverlay(boolean enabled) {
        mEmulationFragment.setLassoOverlay(enabled);
    }

    public boolean isLassoOverlayEnabled() {
        return mEmulationFragment.isLassoOverlayEnabled();
    }

    public void launchMemoryViewer() {
        MemoryActivity.launch(this);
    }

    public void addNetPlayMessage(String msg) {
        mEmulationFragment.addNetPlayMessage(msg);
    }

    public boolean checkPermission(String permission) {
        return checkSelfPermission(permission) == PackageManager.PERMISSION_GRANTED;
    }

    public void updateBackgroundImage() {
        boolean landscape = true;
        int rotation = getDisplayRotation();
        if (rotation == Surface.ROTATION_0 || rotation == Surface.ROTATION_180) {
            landscape = false;
        }
        Bitmap bitmap = mBitmaps.get(landscape ? R.drawable.bg_landscape : R.drawable.bg_portrait);
        if (bitmap != null) {
            bitmap = rotateBitmap(bitmap, 90);
            int width = bitmap.getWidth();
            int height = bitmap.getHeight();
            int[] pixels = new int[width * height];
            bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
            NativeLibrary.SetBackgroundImage(pixels, width, height);
            bitmap.recycle();
        }
    }

    public Bitmap rotateBitmap(Bitmap source, float angle)  {
        Matrix matrix = new Matrix();
        matrix.postRotate(angle);
        return Bitmap.createBitmap(source, 0, 0, source.getWidth(), source.getHeight(), matrix, true);
    }

    public int getSafeInsetLeft() {
        return mSafeInsetLeft;
    }

    public int getSafeInsetTop() {
        return mSafeInsetTop;
    }

    public int getSafeInsetRight() {
        return mSafeInsetRight;
    }

    public int getSafeInsetBottom() {
        return mSafeInsetBottom;
    }

    public int getDisplayRotation() {
        return mDisplayRotation;
    }

    public float getScaleDensity() {
        return mScaledDensity;
    }

    public void launchCheatCode() {
        Intent intent = new Intent(this, EditorActivity.class);
        intent.putExtra(EditorActivity.EXTRA_GAME_ID, mGameId);
        intent.putExtra(EditorActivity.EXTRA_GAME_NAME, mGameName);
        intent.putExtra(EditorActivity.EXTRA_GAME_PATH, mGamePath);
        startActivityForResult(intent, REQUEST_CHEAT_CODE);
    }

    public void rotateScreen() {
        if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT) {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        } else {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        }
    }

    public void updateProgress(String name, long written, long total) {
        mEmulationFragment.updateProgress(name, written, total);
    }

    public Bitmap getInputBitmap(int id, float scale) {
        // Determine the button size based on the smaller screen dimension.
        // This makes sure the buttons are the same size in both portrait and landscape.
        DisplayMetrics dm = getResources().getDisplayMetrics();
        int dimension = (int)(Math.min(dm.widthPixels, dm.heightPixels) * scale);
        Bitmap bitmap = mBitmaps.get(id);
        int dstWidth = bitmap.getWidth();
        int dstHeight = bitmap.getHeight();
        if (dstWidth > dstHeight) {
            dstWidth = dstWidth * dimension / dstHeight;
            dstHeight = dimension;
        } else {
            dstHeight = dstHeight * dimension / dstWidth;
            dstWidth = dimension;
        }
        return Bitmap.createScaledBitmap(mBitmaps.get(id), dstWidth, dstHeight, true);
    }
}
