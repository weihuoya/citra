package org.citra.emu.ui;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;

import android.annotation.SuppressLint;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Intent;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.net.ssl.SSLHandshakeException;

public class AboutActivity extends AppCompatActivity implements SurfaceHolder.Callback {

    @SuppressLint("StaticFieldLeak")
    private class RefreshTask extends AsyncTask<Void, Void, String> {

        @Override
        protected void onPreExecute() {
            mLatestVersionProgress.setIndeterminate(true);
            mLatestVersionProgress.setVisibility(View.VISIBLE);
        }

        @Override
        protected String doInBackground(Void... args) {
            String content = null;
            try {
                URL url = new URL("https://api.github.com/repos/weihuoya/citra/releases/latest");
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(10000);
                connection.setReadTimeout(10000);
                connection.connect();

                int status = connection.getResponseCode();
                if (status == HttpURLConnection.HTTP_OK) {
                    BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                    StringBuilder sb = new StringBuilder();
                    String line;
                    while ((line = reader.readLine()) != null) {
                        sb.append(line);
                        sb.append(System.lineSeparator());
                    }
                    reader.close();
                    connection.disconnect();
                    content = sb.toString();
                } else if (status == HttpURLConnection.HTTP_FORBIDDEN) {
                    String limit = connection.getHeaderField("x-ratelimit-limit");
                    String remaining = connection.getHeaderField("x-ratelimit-remaining");
                    String reset = connection.getHeaderField("x-ratelimit-reset");
                    connection.disconnect();

                    Date date = new Date(Long.parseLong(reset) * 1000);
                    DateFormat dt = SimpleDateFormat.getDateTimeInstance();
                    String dateStr = dt.format(date);
                    Log.v("citra", String.format("limit: %s, remaining: %s, reset: %s", limit, remaining, dateStr));
                    return "server_busy";
                } else {
                    connection.disconnect();
                }
            } catch (SSLHandshakeException e) {
                return "connection_error";
            } catch (Exception e) {
                // ignore
            }

            if (content == null || content.isEmpty()) {
                return null;
            }

            try {
                JSONObject json = new JSONObject(content);
                if (json.has("assets")) {
                    JSONArray assets = json.getJSONArray("assets");
                    if (assets.length() > 0) {
                        JSONObject asset = assets.getJSONObject(0);
                        if (asset.has("browser_download_url")) {
                            return asset.getString("browser_download_url");
                        }
                    }
                }
            } catch (Exception e) {
                // ignore
            }

            return null;
        }

        @Override
        protected void onPostExecute(String args) {
            mLatestVersionProgress.setVisibility(View.INVISIBLE);
            if (args != null) {
                int startIndex = args.lastIndexOf("/");
                int endIndex = args.lastIndexOf(".");
                if (startIndex > 0 && endIndex > startIndex) {
                    String filename = args.substring(startIndex + 1, endIndex);
                    String text = getString(R.string.latest_version);
                    mBtnLatestVersion.setText(String.format("%s(%s)", text, filename));
                    return;
                } else if (args.equals("connection_error")) {
                    String error = getString(R.string.connection_error);
                    String text = getString(R.string.latest_version);
                    mBtnLatestVersion.setText(String.format("%s(%s)", text, error));
                    return;
                } else if (args.equals("server_busy")) {
                    String error = getString(R.string.rate_limit_exceeded);
                    String text = getString(R.string.latest_version);
                    mBtnLatestVersion.setText(String.format("%s(%s)", text, error));
                    return;
                }
            }

            String error = getString(R.string.multiplayer_network_error);
            String text = getString(R.string.latest_version);
            mBtnLatestVersion.setText(String.format("%s(%s)", text, error));
        }
    }

    private ProgressBar mLatestVersionProgress;
    private Button mBtnLatestVersion;
    private String mDonwloadUrl = "https://github.com/weihuoya/citra/releases/latest";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_about);

        Toolbar toolbar = findViewById(R.id.toolbar_main);
        setSupportActionBar(toolbar);

        SurfaceView surfaceView = findViewById(R.id.surface_about);
        surfaceView.getHolder().addCallback(this);

        mLatestVersionProgress = findViewById(R.id.latest_version_progress);
        mBtnLatestVersion = findViewById(R.id.btn_latest_version);
        mBtnLatestVersion.setOnClickListener(view -> {
            if (mDonwloadUrl != null) {
                openUrl(mDonwloadUrl);
            }
        });

        TextView versionInfo = findViewById(R.id.version);
        versionInfo.setText(NativeLibrary.GetBuildDate());

        Button btnOfficialWebsite = findViewById(R.id.btn_official_website);
        btnOfficialWebsite.setOnClickListener(view -> openUrl("https://citra-emu.org/"));

        Button btnOpenWeibo = findViewById(R.id.btn_open_weibo);
        btnOpenWeibo.setOnClickListener(view -> openUrl("https://weibo.com/1725027100"));

        new RefreshTask().execute();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        setDeviceInfo(NativeLibrary.GetDeviceIinfo(holder.getSurface()));
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }

    private void setDeviceInfo(String info) {
        TextView view = findViewById(R.id.device_info);
        SurfaceView surface = findViewById(R.id.surface_about);
        view.setText(info);
        surface.setVisibility(View.GONE);
    }

    private void openUrl(String url) {
        Intent browserIntent = new Intent(Intent.ACTION_VIEW, Uri.parse(url));
        getSystemService(ClipboardManager.class).setPrimaryClip(ClipData.newPlainText("URL", url));
        Toast.makeText(this, R.string.copy_success, Toast.LENGTH_SHORT).show();
        startActivity(browserIntent);
    }
}