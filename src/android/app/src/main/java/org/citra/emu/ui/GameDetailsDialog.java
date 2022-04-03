package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.core.content.pm.ShortcutManagerCompat;
import androidx.fragment.app.DialogFragment;
import androidx.fragment.app.FragmentActivity;
import androidx.fragment.app.FragmentManager;

import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import org.citra.emu.R;
import org.citra.emu.model.GameFile;

public final class GameDetailsDialog extends DialogFragment
{
  private static final String ARG_GAME_PATH = "game_path";
  private static GameFile mGame;

  public static GameDetailsDialog newInstance(String gamePath)
  {
    GameDetailsDialog fragment = new GameDetailsDialog();
    mGame = new GameFile(gamePath);

    Bundle arguments = new Bundle();
    arguments.putString(ARG_GAME_PATH, gamePath);
    fragment.setArguments(arguments);

    return fragment;
  }

  @NonNull
  @Override
  public Dialog onCreateDialog(Bundle savedInstanceState)
  {
    AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
    ViewGroup contents = (ViewGroup) getActivity().getLayoutInflater()
            .inflate(R.layout.dialog_game_details, null);

    // game title
    TextView textGameTitle = contents.findViewById(R.id.text_game_title);
    textGameTitle.setText(mGame.getName());

    // game id
    TextView textGameId = contents.findViewById(R.id.text_game_id);
    textGameId.setText("ID: " + mGame.getId());

    // launch game
    Button buttonLaunchGame = contents.findViewById(R.id.button_launch_game);
    buttonLaunchGame.setOnClickListener(view ->
    {
      this.dismiss();
      EmulationActivity.launch(view.getContext(), mGame);
    });

    // cheat code
    Button buttonCheatCode = contents.findViewById(R.id.button_cheat_code);
    buttonCheatCode.setOnClickListener(view ->
    {
      this.dismiss();
      EditorActivity.launch(getContext(), mGame);
    });

    // shortcut
    FragmentManager fm = ((FragmentActivity) contents.getContext()).getSupportFragmentManager();
    ShortcutDialog shortcutDialog = ShortcutDialog.newInstance(mGame.getPath());
    Button buttonShortcut = contents.findViewById(R.id.button_shortcut);
    if (ShortcutManagerCompat.isRequestPinShortcutSupported(getContext())) {
      buttonShortcut.setVisibility(View.VISIBLE);
    } else {
      buttonShortcut.setVisibility(View.INVISIBLE);
    }
    buttonShortcut.setOnClickListener(view ->
    {
      this.dismiss();
      shortcutDialog.show(fm, "fragment_shortcut");
    });

    // game icon
    ImageView imageGameScreen = contents.findViewById(R.id.image_game_screen);
    imageGameScreen.setImageBitmap(mGame.getIcon());

    builder.setView(contents);
    return builder.create();
  }
}