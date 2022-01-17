package org.citra.emu.model;

public final class GameInfo {
    /**
     * Game regions
     */
    public static final class GameRegion {
        public static final int Japan = 0;
        public static final int NorthAmerica = 1;
        public static final int Europe = 2;
        public static final int Australia = 3;
        public static final int China = 4;
        public static final int Korea = 5;
        public static final int Taiwan = 6;
        public static final int Global = 7;
    }

    public String id;
    public String name;
    public String path;
    public byte[] icon;
    public int region;
}
