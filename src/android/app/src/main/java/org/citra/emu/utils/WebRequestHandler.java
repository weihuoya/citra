package org.citra.emu.utils;

import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

// By default, this implementation of HttpURLConnection requests that servers use gzip compression
// and it automatically decompresses the data for callers of getInputStream().
// getContentLength() return -1
// support Transfer-Encoding: chunked
// read bytes from the response until InputStream.read() returns -1.

public class WebRequestHandler {
    public static WebRequestHandler Create(String urlStr) {
        HttpURLConnection connection;
        InputStream stream;
        try {
            URL url = new URL(urlStr);
            connection = (HttpURLConnection) url.openConnection();
            connection.connect();
            stream = connection.getInputStream();
        } catch (Exception e) {
            Log.v("citra", "WebRequestHandler create failed: " + urlStr, e);
            return null;
        }
        return new WebRequestHandler(connection, stream);
    }

    private final HttpURLConnection mConnection;
    private final InputStream mStream;
    private final byte[] mBuffer;

    public WebRequestHandler(HttpURLConnection connection, InputStream stream) {
        mConnection = connection;
        mStream = stream;
        mBuffer = new byte[1024 * 8];
    }

    public byte[] data() {
        return mBuffer;
    }

    public int read() {
        int size = 0;
        try {
            size = mStream.read(mBuffer);
        } catch (IOException e) {
            Log.v("citra", "WebRequestHandler read error", e);
        }
        return size;
    }

    public void close() {
        try {
            mStream.close();
        } catch (IOException e) {
            // ignore
        }
        mConnection.disconnect();
    }
}
