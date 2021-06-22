package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Intent;
import android.graphics.Bitmap;
import android.net.Uri;
import android.os.Bundle;
import android.provider.Settings;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;

import androidx.annotation.NonNull;
import androidx.core.content.pm.ShortcutInfoCompat;
import androidx.core.content.pm.ShortcutManagerCompat;
import androidx.core.graphics.drawable.IconCompat;
import androidx.fragment.app.DialogFragment;

import org.citra.emu.R;
import org.citra.emu.model.GameFile;

public final class ShortcutDialog extends DialogFragment {
    public static final String EXTRA_GAME_PATH = "GamePath";
    private String mGamePath;
    private GameFile mGame;
    private EditText mName;

    public static ShortcutDialog newInstance(String path) {
        return new ShortcutDialog(path);
    }

    public ShortcutDialog(String path) {
        mGamePath = path;
    }

    @NonNull
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());

        if (savedInstanceState != null) {
            mGamePath = savedInstanceState.getString(EXTRA_GAME_PATH);
        }
        mGame = new GameFile(mGamePath);

        ViewGroup contents =
                (ViewGroup)getActivity().getLayoutInflater().inflate(R.layout.dialog_shortcut, null);

        ImageView imageIcon = contents.findViewById(R.id.image_game_screen);
        imageIcon.setImageBitmap(mGame.getIcon(getContext()));

        mName = contents.findViewById(R.id.text_name);
        mName.setText(mGame.getName());

        Button buttonPermission = contents.findViewById(R.id.button_permission);
        buttonPermission.setOnClickListener(view -> {
            Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
            Uri uri = Uri.fromParts("package", getContext().getPackageName(), null);
            intent.setData(uri);
            startActivity(intent);
        });

        Button buttonConfirm = contents.findViewById(R.id.button_confirm);
        buttonConfirm.setOnClickListener(view -> addShortcut());

        builder.setView(contents);
        return builder.create();
    }

    @Override
    public void onSaveInstanceState(Bundle outState) {
        outState.putString(EXTRA_GAME_PATH, mGamePath);
        super.onSaveInstanceState(outState);
    }

    private void addShortcut() {
        final String id = mGame.getId();
        final String name = mName.getText().toString();
        final Bitmap icon = mGame.getIcon(getContext());
        final Intent intent = new Intent(getContext(), EmulationActivity.class);
        intent.setAction(Intent.ACTION_VIEW);
        intent.putExtra(EmulationActivity.EXTRA_GAME_ID, id);
        intent.putExtra(EmulationActivity.EXTRA_GAME_NAME, name);
        intent.putExtra(EmulationActivity.EXTRA_GAME_PATH, mGamePath);
        ShortcutInfoCompat shortcutInfo = new ShortcutInfoCompat.Builder(getContext(), id)
                .setShortLabel(name)
                .setIcon(IconCompat.createWithBitmap(icon))
                .setIntent(intent)
                .build();
        ShortcutManagerCompat.requestPinShortcut(getContext(), shortcutInfo, null);
        dismiss();
    }
}
