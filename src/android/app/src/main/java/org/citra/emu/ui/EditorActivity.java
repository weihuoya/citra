// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.RecyclerView;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ProgressBar;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import org.citra.emu.R;
import org.citra.emu.utils.DirectoryInitialization;

public final class EditorActivity extends AppCompatActivity {

    private static final String ARG_PROGRAM_ID = "program_id";
    private static final String ARG_PROGRAM_TITLE = "program_title";

    private EditText mEditor;
    private RecyclerView mListView;
    private ProgressBar mProgressBar;

    public static void launch(Context context, String programId, String title) {
        Intent settings = new Intent(context, EditorActivity.class);
        settings.putExtra(ARG_PROGRAM_ID, programId);
        settings.putExtra(ARG_PROGRAM_TITLE, title);
        context.startActivity(settings);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_editor);

        final String programId = getIntent().getStringExtra(ARG_PROGRAM_ID);
        final String title = getIntent().getStringExtra(ARG_PROGRAM_TITLE);

        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        setTitle(title);

        mEditor = findViewById(R.id.code_content);
        mListView = findViewById(R.id.code_list);
        mProgressBar = findViewById(R.id.progress_bar);
        mProgressBar.setVisibility(View.INVISIBLE);

        Button buttonConfirm = findViewById(R.id.button_confirm);
        buttonConfirm.setOnClickListener(view -> {
            saveCheatCode(programId);
            mEditor.clearFocus();
            finish();
        });

        Button buttonCancel = findViewById(R.id.button_cancel);
        buttonCancel.setOnClickListener(view -> {
            mEditor.clearFocus();
            finish();
        });

        loadCheatCode(programId);
        toggleListView(false);
    }

    private void toggleListView(boolean isShowList) {
        if (isShowList) {
            InputMethodManager imm =
                (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
            imm.hideSoftInputFromWindow(getWindow().getDecorView().getWindowToken(), 0);
            mListView.setVisibility(View.VISIBLE);
            mEditor.setVisibility(View.INVISIBLE);
        } else {
            mListView.setVisibility(View.INVISIBLE);
            mEditor.setVisibility(View.VISIBLE);
        }
    }

    private void loadCheatCode(String programId) {
        File cheatFile = DirectoryInitialization.getCheatFile(programId);
        StringBuilder sb = new StringBuilder();
        if (cheatFile != null && cheatFile.exists()) {
            try {
                BufferedReader reader = new BufferedReader(new FileReader(cheatFile));
                String line = reader.readLine();
                while (line != null) {
                    sb.append(line);
                    sb.append(System.lineSeparator());
                    line = reader.readLine();
                }
                reader.close();
            } catch (IOException e) {
                //
            }
        }
        mEditor.setText(sb.toString());
    }

    private void saveCheatCode(String programId) {
        File cheatFile = DirectoryInitialization.getCheatFile(programId);
        String content = mEditor.getText().toString();
        if (content.isEmpty()) {
            cheatFile.delete();
        } else {
            try {
                FileWriter writer = new FileWriter(cheatFile);
                writer.write(content);
                writer.close();
            } catch (IOException e) {
                //
            }
        }
    }
}
