package org.citra.emu.ui;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.net.Uri;
import android.os.Environment;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.EditText;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.FileProvider;

import com.nononsenseapps.filepicker.AbstractFilePickerFragment;
import com.nononsenseapps.filepicker.FilePickerActivity;
import com.nononsenseapps.filepicker.FilePickerFragment;
import java.io.File;
import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

public final class GameFilePickerActivity extends FilePickerActivity implements DialogInterface.OnClickListener {

    private GameFilePickerFragment mFilePickerFragment;

    @Override
    protected AbstractFilePickerFragment<File> getFragment(@Nullable final String startPath,
                                                           final int mode,
                                                           final boolean allowMultiple,
                                                           final boolean allowExistingFile,
                                                           final boolean singleClick) {
        mFilePickerFragment = new GameFilePickerFragment();
        // startPath is allowed to be null. In that case, default folder should be SD-card and not
        // "/"
        mFilePickerFragment.setArgs(startPath != null ?
                startPath : Environment.getExternalStorageDirectory().getPath(),
                mode, allowMultiple, allowExistingFile, singleClick);
        return mFilePickerFragment;
    }

    public static class GameFilePickerFragment extends FilePickerFragment {
        @NonNull
        @Override
        public Uri toUri(@NonNull final File file) {
            return FileProvider.getUriForFile(
                getContext(),
                getContext().getApplicationContext().getPackageName() + ".filesprovider", file);
        }

        @Override
        protected boolean isItemVisible(final File file) {
            if (file.isHidden())
                return false;
            if (file.isDirectory())
                return true;
            if (mode == MODE_FILE)
                return file.getName().endsWith(".cia");
            return NativeLibrary.isValidFile(file.getName());
        }

        public String getCurrentPath() {
            return mCurrentPath.toString();
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_file_picker, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_current_directory:
                showEditorDialog();
                return true;
        }

        return false;
    }

    @Override
    public void onClick(DialogInterface dialog, int which) {
        EditText editor = ((AlertDialog)dialog).findViewById(R.id.setting_editor);
        String text = editor.getText().toString();
        File path = new File(text);
        if (path.isDirectory()) {
            mFilePickerFragment.goToDir(path);
        } else {
            Toast.makeText(this, R.string.nnf_need_valid_filename, Toast.LENGTH_SHORT).show();
        }
    }

    private void showEditorDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);

        LayoutInflater inflater = LayoutInflater.from(this);
        View view = inflater.inflate(R.layout.dialog_editor, null);

        builder.setTitle(R.string.current_directory);
        builder.setView(view);
        builder.setPositiveButton(android.R.string.ok, this);
        builder.show();

        EditText editor = view.findViewById(R.id.setting_editor);
        editor.setText(mFilePickerFragment.getCurrentPath());
        editor.requestFocus();
    }
}
