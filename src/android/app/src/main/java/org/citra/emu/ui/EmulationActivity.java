package org.citra.emu.ui;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.os.Bundle;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.io.InputStream;
import java.lang.ref.WeakReference;
import java.util.List;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.utils.ControllerMappingHelper;

public final class EmulationActivity extends AppCompatActivity {
    private static final String EXTRA_GAME_ID = "GameId";
    private static final String EXTRA_GAME_NAME = "GameName";
    private static final String EXTRA_GAME_PATH = "GamePath";
    public static final int REQUEST_PICK_IMAGE = 1;
    public static final int REQUEST_CHEAT_CODE = 2;

    private static WeakReference<EmulationActivity> sInstance = new WeakReference<>(null);

    private String mGameId;
    private String mGameName;
    private String mGamePath;
    private boolean mMenuVisible;
    private EmulationFragment mEmulationFragment;

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

        hideSystemUI();

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

    private void hideSystemUI() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
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
            // Handling the case where the back button is pressed.
            if (button == KeyEvent.KEYCODE_BACK) {
                onBackPressed();
                return true;
            }
            // Normal key events.
            action = NativeLibrary.ButtonState.PRESSED;
            break;
        case KeyEvent.ACTION_UP:
            action = NativeLibrary.ButtonState.RELEASED;
            break;
        default:
            return false;
        }

        if (input != null)
            return NativeLibrary.KeyEvent(button, action);
        else
            return false;
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event) {
        if (mMenuVisible) {
            return false;
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

    public void launchMemoryViewer() {
        MemoryActivity.launch(this);
    }

    public void launchCheatCode() {
        Intent intent = new Intent(this, EditorActivity.class);
        intent.putExtra(EditorActivity.ARG_PROGRAM_ID, mGameId);
        intent.putExtra(EditorActivity.ARG_PROGRAM_TITLE, mGameName);
        startActivityForResult(intent, REQUEST_CHEAT_CODE);
    }

    public void rotateScreen() {
        if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT) {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        } else {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        }
    }
}
