package org.citra.emu.settings;

import androidx.annotation.Nullable;

public enum MenuTag {
    CONFIG("Config"),
    INPUT("Input");

    private String mTag;
    private int mSubType;

    MenuTag(String tag) {
        mTag = tag;
        mSubType = -1;
    }

    MenuTag(String tag, int subtype) {
        mTag = tag;
        mSubType = subtype;
    }

    @Nullable
    public static MenuTag getMenuTag(String menuTagStr) {
        if (menuTagStr == null || menuTagStr.isEmpty()) {
            return null;
        }
        String tag = menuTagStr;
        int subtype = -1;
        int sep = menuTagStr.indexOf('|');
        if (sep != -1) {
            tag = menuTagStr.substring(0, sep);
            subtype = Integer.parseInt(menuTagStr.substring(sep + 1));
        }
        return getMenuTag(tag, subtype);
    }

    private static MenuTag getMenuTag(String tag, int subtype) {
        for (MenuTag menuTag : MenuTag.values()) {
            if (menuTag.mTag.equals(tag) && menuTag.mSubType == subtype)
                return menuTag;
        }

        throw new IllegalArgumentException("You are asking for a menu that is not available or "
                                           + "passing a wrong subtype");
    }

    @Override
    public String toString() {
        if (mSubType != -1) {
            return mTag + "|" + mSubType;
        }
        return mTag;
    }

    public String getTag() {
        return mTag;
    }

    public int getSubType() {
        return mSubType;
    }
}
