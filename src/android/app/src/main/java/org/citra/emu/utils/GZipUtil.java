package org.citra.emu.utils;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;

public class GZipUtil {

    public static boolean isGZipped(File f) {
        int magic = 0;
        try {
            RandomAccessFile raf = new RandomAccessFile(f, "r");
            magic = raf.read() & 0xff | ((raf.read() << 8) & 0xff00);
            raf.close();
        } catch (Throwable e) {
            e.printStackTrace(System.err);
        }
        return magic == GZIPInputStream.GZIP_MAGIC;
    }

    public static byte[] compress(String data) {
        byte[] compressed = null;
        ByteArrayOutputStream bos = new ByteArrayOutputStream(data.length());
        try {
            GZIPOutputStream gzip = new GZIPOutputStream(bos);
            gzip.write(data.getBytes());
            gzip.close();
            compressed = bos.toByteArray();
            bos.close();
        } catch (IOException e) {
            // ignore
        }
        return compressed;
    }

    public static String decompress(byte[] compressed) {
        int n;
        char[] buffer = new char[1024 * 8];
        StringBuilder sb = new StringBuilder();
        ByteArrayInputStream bis = new ByteArrayInputStream(compressed);
        try {
            GZIPInputStream gis = new GZIPInputStream(bis);
            InputStreamReader reader = new InputStreamReader(gis, StandardCharsets.UTF_8);
            while (-1 != (n = reader.read(buffer)) ) {
                sb.append(buffer, 0, n);
            }
        } catch (IOException e) {
            // ignore
        }
        return sb.toString();
    }

    public static String readAllText(File f) {
        int n;
        char[] buffer = new char[1024 * 8];
        StringBuilder sb = new StringBuilder();
        try {
            FileInputStream fin = new FileInputStream(f);
            GZIPInputStream gis = new GZIPInputStream(fin);
            InputStreamReader reader = new InputStreamReader(gis, StandardCharsets.UTF_8);
            while (-1 != (n = reader.read(buffer)) ) {
                sb.append(buffer, 0, n);
            }
        } catch (IOException e) {
            // ignore
        }
        return sb.toString();
    }

    public static void writeAllText(File f, String text) {
        try {
            FileOutputStream fout = new FileOutputStream(f);
            GZIPOutputStream gzip = new GZIPOutputStream(fout);
            gzip.write(text.getBytes());
            gzip.close();
            fout.close();
        } catch (IOException e) {
            // ignore
        }
    }
}
