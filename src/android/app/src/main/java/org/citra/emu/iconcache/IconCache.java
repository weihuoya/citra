package org.citra.emu.iconcache;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteException;
import android.util.Log;

import org.citra.emu.model.GameInfo;

import java.util.HashMap;
import java.util.Map;

public class IconCache {
    private static final int INITIAL_ICON_CACHE_CAPACITY = 50;
    private final Map<String, GameInfo> mCache;
    protected IconDB mIconDb;

    public IconCache(Context context) {
        mCache = new HashMap<>(INITIAL_ICON_CACHE_CAPACITY);
        mIconDb = new IconDB(context, "icons.db");
    }

    public void clearCache() {
        mIconDb.clear();
        mCache.clear();
    }

    public void remove(String path) {
        mCache.remove(path);
        mIconDb.delete(IconDB.COLUMN_PATH + " = ?", new String[]{path});
    }

    public GameInfo getEntry(String path) {
        GameInfo entry = mCache.get(path);
        if (entry != null) {
            return entry;
        }

        try (Cursor c = mIconDb.query(IconDB.COLUMNS, IconDB.COLUMN_PATH + " = ?", new String[]{path})) {
            if (c.moveToNext()) {
                entry = new GameInfo();
                entry.path = path;
                entry.id = c.getString(0);
                entry.name = c.getString(1);
                entry.region = c.getInt(2);
                entry.icon = c.getBlob(3);
                return entry;
            }
        } catch (SQLiteException e) {
            Log.e("citra", "Error reading icon cache", e);
        }

        return null;
    }

    public void addIconToDB(GameInfo game) {
        if (game.icon == null || game.path == null) {
            return;
        }
        ContentValues values = new ContentValues();
        values.put(IconDB.COLUMN_APPID, game.id);
        values.put(IconDB.COLUMN_NAME, game.name);
        values.put(IconDB.COLUMN_PATH, game.path);
        values.put(IconDB.COLUMN_REGION, game.region);
        values.put(IconDB.COLUMN_ICON, game.icon);
        mIconDb.insertOrReplace(values);
        mCache.put(game.path, game);
    }

    /**
     * Cache class to store the actual entries on disk
     */
    private static final class IconDB extends SQLiteCacheHelper {
        private static final int RELEASE_VERSION = 27;
        public static final String TABLE_NAME = "icons";
        public static final String COLUMN_APPID = "appid";
        public static final String COLUMN_NAME = "name";
        public static final String COLUMN_REGION = "region";
        public static final String COLUMN_ICON = "icon";
        public static final String COLUMN_PATH = "path";
        public static final String[] COLUMNS = new String[] {
                COLUMN_APPID, COLUMN_NAME, COLUMN_REGION, COLUMN_ICON };
        public IconDB(Context context, String dbFileName) {
            super(context, dbFileName, (RELEASE_VERSION << 16) + 48, TABLE_NAME);
        }
        @Override
        protected void onCreateTable(SQLiteDatabase db) {
            db.execSQL("CREATE TABLE IF NOT EXISTS " + TABLE_NAME + " ("
                    + COLUMN_REGION + " INTEGER NOT NULL DEFAULT 0, "
                    + COLUMN_ICON + " BLOB, "
                    + COLUMN_NAME + " TEXT, "
                    + COLUMN_APPID + " TEXT NOT NULL, "
                    + COLUMN_PATH + " TEXT NOT NULL, "
                    + "PRIMARY KEY (" + COLUMN_PATH + ") "
                    + ");");
        }
    }
}
