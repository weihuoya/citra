package org.citra.emu.ui;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;

import android.content.Intent;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;

import org.citra.emu.R;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class AboutActivity extends AppCompatActivity {

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
                    content = sb.toString();
                }

                connection.disconnect();
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
            if (args != null) {
                int startIndex = args.lastIndexOf("/");
                int endIndex = args.lastIndexOf(".");
                if (startIndex > 0 && endIndex > startIndex) {
                    String filename = args.substring(startIndex + 1, endIndex);
                    String text = getString(R.string.latest_version);
                    mBtnLatestVersion.setText(String.format("%s(%s)", text, filename));
                }
                mDonwloadUrl = args;
            } else {
                String error = getString(R.string.multiplayer_network_error);
                String text = getString(R.string.latest_version);
                mBtnLatestVersion.setText(String.format("%s(%s)", text, error));
            }
            mLatestVersionProgress.setVisibility(View.INVISIBLE);
        }
    }

    private ProgressBar mLatestVersionProgress;
    private Button mBtnLatestVersion;
    private String mDonwloadUrl;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_about);

        Toolbar toolbar = findViewById(R.id.toolbar_main);
        setSupportActionBar(toolbar);

        mLatestVersionProgress = findViewById(R.id.latest_version_progress);
        mBtnLatestVersion = findViewById(R.id.btn_latest_version);
        mBtnLatestVersion.setOnClickListener(view -> {
            if (mDonwloadUrl != null) {
                openUrl(mDonwloadUrl);
            }
        });

        Button btnOfficialWebsite = findViewById(R.id.btn_official_website);
        btnOfficialWebsite.setOnClickListener(view -> openUrl("https://citra-emu.org/"));

        Button btnOpenWeibo = findViewById(R.id.btn_open_weibo);
        btnOpenWeibo.setOnClickListener(view -> openUrl("https://weibo.com/1725027100"));

        new RefreshTask().execute();
    }

    private void openUrl(String url) {
        Intent browserIntent = new Intent(Intent.ACTION_VIEW, Uri.parse(url));
        startActivity(browserIntent);
    }
}