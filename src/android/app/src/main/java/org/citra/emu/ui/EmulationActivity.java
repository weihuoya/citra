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
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;

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
    private View mDecorView;
    private boolean mMenuVisible;
    private boolean mStopEmulation;
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

        // Get a handle to the Window containing the UI.
        mDecorView = getWindow().getDecorView();
        mDecorView.setOnSystemUiVisibilityChangeListener(visibility -> {
            if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                // Go back to immersive fullscreen mode in 3s
                Handler handler = new Handler(getMainLooper());
                handler.postDelayed(this ::enableFullscreenImmersive, 3000 /* 3s */);
            }
        });
        // fullscreen on create
        enableFullscreenImmersive();
        mStopEmulation = false;

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
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_emulation, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case R.id.menu_mem_region:
            MemoryActivity.launch(this);
            break;

        case R.id.menu_screen_rotation:
            if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT) {
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            } else {
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
            }
            break;

        case R.id.menu_cheat_code: {
            Intent intent = new Intent(this, EditorActivity.class);
            intent.putExtra(EditorActivity.ARG_PROGRAM_ID, mGameId);
            intent.putExtra(EditorActivity.ARG_PROGRAM_TITLE, mGameName);
            startActivityForResult(intent, REQUEST_CHEAT_CODE);
            break;
        }

        case R.id.menu_running_setting:
            RunningSettingDialog.newInstance().show(getSupportFragmentManager(),
                                                    "RunningSettingDialog");
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
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_IMMERSIVE);
    }

    private void disableFullscreenImmersive() {
        mMenuVisible = true;
        mDecorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
                                         View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
                                         View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
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

    public void refreshControls() {
        mEmulationFragment.refreshControls();
    }
}
