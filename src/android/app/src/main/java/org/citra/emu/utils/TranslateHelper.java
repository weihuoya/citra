package org.citra.emu.utils;

import android.graphics.Bitmap;
import android.util.Base64;
import android.util.Log;

import org.citra.emu.NativeLibrary;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

public final class TranslateHelper {

    public static final int ERROR_SUCCESS = 0;

    public static final int ERROR_BAIDUTOKEN_NET_ERROR = 1;
    public static final int ERROR_BAIDUTOKEN_INVALID_CLIENT = 2;
    public static final int ERROR_BAIDUTOKEN_INVALID_SECRET = 3;
    public static final int ERROR_BAIDUTOKEN_INVALID_CONTENT = 4;
    public static final int ERROR_BAIDUTOKEN_UNKNOWN_ERROR = 5;

    public static final int ERROR_BAIDUOCR_NET_ERROR = 11;
    public static final int ERROR_BAIDUOCR_MISMATCH_API = 22;
    public static final int ERROR_BAIDUOCR_OUT_OF_USAGE = 33;
    public static final int ERROR_BAIDUOCR_TRY_AGAIN = 44;
    public static final int ERROR_BAIDUOCR_TOKEN_OVERDUE = 55;
    public static final int ERROR_BAIDUOCR_TOO_SMALL = 66;
    public static final int ERROR_BAIDUOCR_INVALID_CONTENT = 77;
    public static final int ERROR_BAIDUOCR_EMPTY_RESULT = 88;
    public static final int ERROR_BAIDUOCR_UNKNOWN_ERROR = 99;

    public static final int ERROR_YOUDAO_NET_ERROR = 111;
    public static final int ERROR_YOUDAO_INVALID_CONTENT = 222;
    public static final int ERROR_YOUDAO_UNKNOWN_ERROR = 333;

    public static final int ERROR_YEEKIT_NET_ERROR = 1111;
    public static final int ERROR_YEEKIT_INVALID_CONTENT = 2222;
    public static final int ERROR_YEEKIT_UNKNOWN_ERROR = 3333;

    public static final int ERROR_GOOGLE_NET_ERROR = 11111;
    public static final int ERROR_GOOGLE_INVALID_CONTENT = 22222;
    public static final int ERROR_GOOGLE_UNKNOWN_ERROR = 33333;

    public static final int LANGUAGE_AUTO = 0;
    public static final int LANGUAGE_JPN = 1;
    public static final int LANGUAGE_ENG = 2;
    public static final int LANGUAGE_KOR = 3;

    public static final int NONE_TRANSLATE = 0;
    public static final int GOOGLE_TRANSLATE = 1;
    public static final int YOUDAO_TRANSLATE = 2;
    public static final int YEEKIT_TRANSLATE = 3;

    // baidu ocr
    public static final String BaiduOCRLanguageJPN = "JAP";
    public static final String BaiduOCRLanguageENG = "ENG";
    public static final String BaiduOCRLanguageKOR = "KOR";

    // yaodao translate
    private static final String YoudaoLanguageAUTO = "AUTO";
    private static final String YoudaoLanguageJPN = "ja";
    private static final String YoudaoLanguageENG = "en";
    private static final String YoudaoLanguageKOR = "ko";

    // yeekit translate
    private static final String YeekitLanguageAUTO = "nen";
    private static final String YeekitLanguageJPN = "nja";
    private static final String YeekitLanguageENG = "nen";
    private static final String YeekitLanguageKOR = "nko";

    // google translate
    private static final String GoogleLanguageAUTO = "auto";
    private static final String GoogleLanguageJPN = "ja";
    private static final String GoogleLanguageENG = "en";
    private static final String GoogleLanguageKOR = "ko";

    public static boolean IsRunning = false;
    public static boolean ShowOCRResults = false;
    public static int Service = TranslateHelper.GOOGLE_TRANSLATE;
    public static int Language = TranslateHelper.LANGUAGE_AUTO;

    public static String BaiduOCRKey = null;
    public static String BaiduOCRSecret = null;
    public static String BaiduOCRToken = null;
    public static String BaiduOCRLanguage = null;
    public static boolean BaiduOCRHighPrecision = false;
    public static ArrayList<String> BaiduOCRResults = null;
    public static ArrayList<String> TranslateResults = null;

    public static void Initialize(String key, String secret) {
        if (IsRunning) {
            return;
        }

        if (BaiduOCRToken != null && key.equals(BaiduOCRKey) && secret.equals(BaiduOCRSecret)) {
            return;
        }

        String lan = Locale.getDefault().getLanguage();
        if (!lan.equals("zh")) {
            return;
        }

        IsRunning = true;
        BaiduOCRToken = null;
        BaiduOCRKey = key;
        BaiduOCRSecret = secret;
        new Thread(() -> {
            int error = TranslateHelper.RequestBaiduToken();
            if (error == TranslateHelper.ERROR_BAIDUTOKEN_NET_ERROR) {
                Log.v("citra", "Network Error!");
            } else if (error == TranslateHelper.ERROR_BAIDUTOKEN_INVALID_CLIENT) {
                Log.v("citra", "Key Error!");
            } else if (error == TranslateHelper.ERROR_BAIDUTOKEN_INVALID_SECRET) {
                Log.v("citra", "Secret Error!");
            } else if (error == TranslateHelper.ERROR_BAIDUTOKEN_INVALID_CONTENT) {
                Log.v("citra", "Invalid Response!");
            } else if (error == TranslateHelper.ERROR_BAIDUTOKEN_UNKNOWN_ERROR) {
                Log.v("citra", "Unknown Error!");
            }
            TranslateHelper.IsRunning = false;
        }).start();
    }

    public static int RequestBaiduToken() {
        String key = TranslateHelper.BaiduOCRKey;
        String secret = TranslateHelper.BaiduOCRSecret;
        String urlStr = "https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=%s&client_secret=%s";
        String content = null;

        Log.v("citra", "RequestBaiduToken");

        if (key.isEmpty() || secret.isEmpty()) {
            return ERROR_BAIDUTOKEN_INVALID_CLIENT;
        }

        try {
            URL url = new URL(String.format(urlStr, key, secret));
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
            // TODO
        }

        if (content == null || content.isEmpty()) {
            return ERROR_BAIDUTOKEN_NET_ERROR;
        }

        try {
            JSONObject json = new JSONObject(content);
            if (json.has("access_token")) {
                TranslateHelper.BaiduOCRToken = json.getString("access_token");
                return ERROR_SUCCESS;
            } else if (json.has("error_description")) {
                String error_description = json.getString("error_description");
                if ("unknown client id".equals(error_description)) {
                    return ERROR_BAIDUTOKEN_INVALID_CLIENT;
                } else if ("Client authentication failed".equals(error_description)) {
                    return ERROR_BAIDUTOKEN_INVALID_SECRET;
                } else {
                    int error = json.getInt("error");
                    Log.v("citra", String.format("Error(%d): %s", error, error_description));
                    return ERROR_BAIDUTOKEN_UNKNOWN_ERROR;
                }
            }
        } catch (Exception e) {
            // TODO
        }

        Log.v("citra", content);
        return ERROR_BAIDUTOKEN_INVALID_CONTENT;
    }

    public static int RequestBaiduOCR(Bitmap image) {
        String urlStr = null;
        String content = null;
        if (TranslateHelper.BaiduOCRHighPrecision) {
            urlStr = "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic?access_token=%s";
        } else {
            urlStr = "https://aip.baidubce.com/rest/2.0/ocr/v1/general_basic?access_token=%s";
        }

        TranslateHelper.BaiduOCRLanguage = null;
        if (TranslateHelper.Language == TranslateHelper.LANGUAGE_JPN) {
            TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageJPN;
        } else if (TranslateHelper.Language == TranslateHelper.LANGUAGE_ENG) {
            TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageENG;
        } else if (TranslateHelper.Language == TranslateHelper.LANGUAGE_KOR) {
            TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageKOR;
        }

        Log.v("citra", "RequestBaiduOCR");

        int width = image.getWidth();
        int height = image.getHeight();
        ByteArrayOutputStream pixels = new ByteArrayOutputStream(width * height * 4);
        image.compress(Bitmap.CompressFormat.JPEG, 90, pixels);
        String data = Base64.encodeToString(pixels.toByteArray(), Base64.NO_WRAP);

        try {
            URL url = new URL(String.format(urlStr, TranslateHelper.BaiduOCRToken));
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
            connection.setDoOutput(true);
            connection.setDoInput(true);
            connection.setUseCaches(false);

            connection.connect();
            DataOutputStream output = new DataOutputStream(connection.getOutputStream());
            data = URLEncoder.encode(data, "UTF-8");
            String payload;
            if (TranslateHelper.BaiduOCRLanguage != null) {
                payload = String.format("image=%s&language_type=%s", data, TranslateHelper.BaiduOCRLanguage);
            } else {
                payload = "detect_language=true&image=" + data;
            }
            output.write(payload.getBytes(StandardCharsets.UTF_8));
            output.flush();
            output.close();

            int status = connection.getResponseCode();
            if (status == HttpURLConnection.HTTP_OK) {
                BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line);
                }
                reader.close();
                content = sb.toString();
            }

            connection.disconnect();
        } catch (Exception e) {
            // TODO
        }

        if (content == null || content.isEmpty()) {
            return ERROR_BAIDUOCR_NET_ERROR;
        }

        try {
            JSONObject json = new JSONObject(content);
            if (json.has("words_result")) {
                Log.v("citra", json.toString());
                JSONArray results = json.getJSONArray("words_result");
                int totalLength = 0;
                TranslateHelper.BaiduOCRResults = new ArrayList<>();
                for (int i = 0; i < results.length(); ++i) {
                    String text = results.getJSONObject(i).getString("words");
                    totalLength += text.length();
                    TranslateHelper.BaiduOCRResults.add(text);
                }

                if (json.has("language")) {
                    int lan = json.getInt("language");
                    if (lan == 0) {
                        TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageENG;
                    } else if (lan == 1) {
                        TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageJPN;
                    } else if (lan == 2) {
                        TranslateHelper.BaiduOCRLanguage = TranslateHelper.BaiduOCRLanguageKOR;
                    } else if (lan == 3) {
                        // Chinese
                    }
                }

                return totalLength == 0 ? ERROR_BAIDUOCR_EMPTY_RESULT : ERROR_SUCCESS;
            } else {
                int error = json.getInt("error_code");
                if (error == 6) {
                    return ERROR_BAIDUOCR_MISMATCH_API;
                } else if (error == 17) {
                    return ERROR_BAIDUOCR_OUT_OF_USAGE;
                } else if (error == 18) {
                    return ERROR_BAIDUOCR_TRY_AGAIN;
                } else if (error == 111) {
                    return ERROR_BAIDUOCR_TOKEN_OVERDUE;
                } else if (error == 216202) {
                    return ERROR_BAIDUOCR_TOO_SMALL;
                } else {
                    String msg = json.getString("error_msg");
                    Log.v("citra", String.format("Error(%d): %s", error, msg));
                    return ERROR_BAIDUOCR_UNKNOWN_ERROR;
                }
            }
        } catch (Exception e) {
            // TODO
        }

        Log.v("citra", content);
        return ERROR_BAIDUOCR_INVALID_CONTENT;
    }

    public static int RequestYoudao() {
        String content = null;
        String urlStr = "https://fanyi.youdao.com/translate?smartresult=dict&smartresult=rule";
        String userAgent = "Mozilla/5.0 (Windows NT 10.0; Win64; x64)AppleWebKit/537.36 (KHTML, like Gecko) Chrome/78.0.3904.97 Safari/537.36";
        String payload = "i=%s&from=%s&to=zh-CHS&smartresult=dict&client=fanyideskweb&doctype=json&version=2.1&keyfrom=fanyi.web&action=FY_BY_REALTIME&typoResult=false";

        StringBuilder textSB = new StringBuilder();
        for (String text : TranslateHelper.BaiduOCRResults) {
            textSB.append(text);
            textSB.append(' ');
        }
        String text = textSB.toString();
        String language = TranslateHelper.YoudaoLanguageAUTO;
        if (TranslateHelper.BaiduOCRLanguage == null) {
            language = TranslateHelper.YoudaoLanguageAUTO;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageENG)) {
            language = TranslateHelper.YoudaoLanguageENG;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageJPN)) {
            language = TranslateHelper.YoudaoLanguageJPN;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageKOR)) {
            language = TranslateHelper.YoudaoLanguageKOR;
        }

        Log.v("citra", "RequestYoudao text: " + text);

        try {
            URL url = new URL(urlStr);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
            connection.setRequestProperty("User-Agent", userAgent);
            connection.setDoOutput(true);
            connection.setDoInput(true);
            connection.setUseCaches(false);

            connection.connect();
            DataOutputStream output = new DataOutputStream(connection.getOutputStream());
            payload = String.format(payload, URLEncoder.encode(text, "UTF-8"), language);
            output.write(payload.getBytes(StandardCharsets.UTF_8));
            output.flush();
            output.close();

            int status = connection.getResponseCode();
            if (status == HttpURLConnection.HTTP_OK) {
                BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line);
                }
                reader.close();
                content = sb.toString();
            }

            connection.disconnect();

        } catch (Exception e) {
            // TODO
        }

        if (content == null || content.isEmpty()) {
            return ERROR_YOUDAO_NET_ERROR;
        }

        try {
            JSONObject json = new JSONObject(content);
            if (json.has("translateResult")) {
                JSONArray translateResults = json.getJSONArray("translateResult");
                JSONArray results = translateResults.getJSONArray(0);
                TranslateHelper.TranslateResults = new ArrayList<>();
                for (int i = 0; i < results.length(); ++i) {
                    TranslateHelper.TranslateResults.add(results.getJSONObject(i).getString("tgt"));
                }
                return ERROR_SUCCESS;
            } else {
                Log.v("citra", content);
                return ERROR_YOUDAO_UNKNOWN_ERROR;
            }
        } catch (Exception e) {
            // TODO
        }

        Log.v("citra", content);
        return ERROR_YOUDAO_INVALID_CONTENT;
    }

    public static int RequestYeekit() {
        String content = null;
        String urlStr = "https://www.yeekit.com/site/dotranslate";
        String userAgent = "Mozilla/5.0 (Windows NT 10.0; Win64; x64)AppleWebKit/537.36 (KHTML, like Gecko) Chrome/78.0.3904.97 Safari/537.36";
        String payload = "content%%5B%%5D=%s&sourceLang=%s&targetLang=nzh";

        StringBuilder textSB = new StringBuilder();
        for (String text : TranslateHelper.BaiduOCRResults) {
            textSB.append(text);
            textSB.append(' ');
        }
        String text = textSB.toString();
        String language = TranslateHelper.GoogleLanguageAUTO;
        if (TranslateHelper.BaiduOCRLanguage == null) {
            language = TranslateHelper.YeekitLanguageAUTO;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageENG)) {
            language = TranslateHelper.YeekitLanguageENG;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageJPN)) {
            language = TranslateHelper.YeekitLanguageJPN;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageKOR)) {
            language = TranslateHelper.YeekitLanguageKOR;
        }

        Log.v("citra", "RequestYeekit text: " + text);

        try {
            URL url = new URL(urlStr);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
            connection.setRequestProperty("User-Agent", userAgent);
            connection.setDoOutput(true);
            connection.setDoInput(true);
            connection.setUseCaches(false);

            connection.connect();
            DataOutputStream output = new DataOutputStream(connection.getOutputStream());
            payload = String.format(payload, URLEncoder.encode(text, "UTF-8"), language);
            output.write(payload.getBytes(StandardCharsets.UTF_8));
            output.flush();
            output.close();

            int status = connection.getResponseCode();
            if (status == HttpURLConnection.HTTP_OK) {
                BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line);
                }
                reader.close();
                content = sb.toString();
            }

            connection.disconnect();

        } catch (Exception e) {
            // TODO
        }

        if (content == null || content.isEmpty()) {
            return ERROR_YEEKIT_NET_ERROR;
        }

        try {
            content = content.substring(2, content.length() - 2).replace("\\n", "").replace("\\\"", "\"");
            JSONObject json = new JSONObject(content);
            JSONArray results = json.getJSONArray("translation").getJSONObject(0).getJSONArray("translated").getJSONObject(0).getJSONArray("translation list").getJSONArray(0);
            TranslateHelper.TranslateResults = new ArrayList<>();
            for (int i = 0; i < results.length(); ++i) {
                TranslateHelper.TranslateResults.add(results.getString(i));
            }
            return ERROR_SUCCESS;
        } catch (Exception e) {
            // TODO
        }

        Log.v("citra", content);
        return ERROR_YEEKIT_INVALID_CONTENT;
    }

    public static int RequestGoogle() {
        String content = null;
        String urlStr = "https://translate.google.cn/translate_a/single?client=webapp&sl=%s&tl=zh-CN&hl=zh-CN&dt=at&dt=bd&dt=ex&dt=ld&dt=md&dt=qca&dt=rw&dt=rm&dt=sos&dt=ss&dt=t&otf=2&ssel=0&tsel=0&kc=1&tk=%s&q=%s";
        String userAgent = "Mozilla/5.0 (Windows NT 10.0; Win64; x64)AppleWebKit/537.36 (KHTML, like Gecko) Chrome/78.0.3904.97 Safari/537.36";

        StringBuilder textSB = new StringBuilder();
        for (String text : TranslateHelper.BaiduOCRResults) {
            textSB.append(text);
            textSB.append(' ');
        }
        String text = textSB.toString();
        String language = TranslateHelper.GoogleLanguageAUTO;
        if (TranslateHelper.BaiduOCRLanguage == null) {
            language = TranslateHelper.GoogleLanguageAUTO;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageENG)) {
            language = TranslateHelper.GoogleLanguageENG;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageJPN)) {
            language = TranslateHelper.GoogleLanguageJPN;
        } else if (TranslateHelper.BaiduOCRLanguage.equals(TranslateHelper.BaiduOCRLanguageKOR)) {
            language = TranslateHelper.GoogleLanguageKOR;
        }

        Log.v("citra", "RequestGoogle text: " + text);

        try {
            URL url = new URL(String.format(urlStr, language, GoogleTranslateToken(text),  URLEncoder.encode(text, "UTF-8")));
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestProperty("User-Agent", userAgent);
            connection.connect();

            int status = connection.getResponseCode();
            if (status == HttpURLConnection.HTTP_OK) {
                BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line);
                }
                reader.close();
                content = sb.toString();
            }

            connection.disconnect();

        } catch (Exception e) {
            // TODO
        }

        if (content == null || content.isEmpty()) {
            return ERROR_GOOGLE_NET_ERROR;
        }

        try {
            JSONArray json = new JSONArray(content).getJSONArray(0);
            TranslateHelper.TranslateResults = new ArrayList<>();
            for (int i = 0; i < json.length() - 1; ++i) {
                TranslateHelper.TranslateResults.add(json.getJSONArray(i).getString(0));
            }
            return ERROR_SUCCESS;
        } catch (Exception e) {
            // TODO
        }

        Log.v("citra", content);
        return ERROR_GOOGLE_INVALID_CONTENT;
    }

    public static native String GoogleTranslateToken(String text);
}
