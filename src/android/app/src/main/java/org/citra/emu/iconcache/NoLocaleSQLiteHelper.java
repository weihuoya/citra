package org.citra.emu.iconcache;

import static android.database.sqlite.SQLiteDatabase.NO_LOCALIZED_COLLATORS;
import android.content.Context;
import android.content.ContextWrapper;
import android.database.DatabaseErrorHandler;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteDatabase.CursorFactory;
import android.database.sqlite.SQLiteDatabase.OpenParams;
import android.database.sqlite.SQLiteOpenHelper;
import android.os.Build;

public abstract class NoLocaleSQLiteHelper extends SQLiteOpenHelper {
    private static final boolean ATLEAST_P =
            Build.VERSION.SDK_INT >= Build.VERSION_CODES.P;
    public NoLocaleSQLiteHelper(Context context, String name, int version) {
        super(ATLEAST_P ? context : new NoLocalContext(context), name, null, version);
        if (ATLEAST_P) {
            setOpenParams(new OpenParams.Builder().addOpenFlags(NO_LOCALIZED_COLLATORS).build());
        }
    }
    private static class NoLocalContext extends ContextWrapper {
        public NoLocalContext(Context base) {
            super(base);
        }
        @Override
        public SQLiteDatabase openOrCreateDatabase(
                String name, int mode, CursorFactory factory, DatabaseErrorHandler errorHandler) {
            return super.openOrCreateDatabase(
                    name, mode | Context.MODE_NO_LOCALIZED_COLLATORS, factory, errorHandler);
        }
    }
}
