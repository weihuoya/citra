// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.content.Context;
import android.content.Intent;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import androidx.appcompat.widget.Toolbar;

import com.nononsenseapps.filepicker.DividerItemDecoration;

import org.citra.emu.R;
import org.citra.emu.utils.CitraDirectory;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class SystemFilesActivity extends AppCompatActivity {

    public static void launch(Context context) {
        Intent settings = new Intent(context, SystemFilesActivity.class);
        context.startActivity(settings);
    }

    private ProgressBar mProgressBar;
    private RecyclerView mListView;
    private FileAdapter mAdapter;
    private List<String> mItems;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_system_files);

        Toolbar toolbar = findViewById(R.id.toolbar_main);
        setSupportActionBar(toolbar);

        mProgressBar = findViewById(R.id.progress_bar);
        mListView = findViewById(R.id.grid_files);
        mAdapter = new FileAdapter();
        mListView.setAdapter(mAdapter);
        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        mListView.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new GridLayoutManager(this, 1);
        mListView.setLayoutManager(layoutManager);

        loadSystemFiles();
    }

    protected void loadSystemFiles() {
        mItems = new ArrayList<>();

        File root = getNandTitlePath();
        File[] files = root.listFiles();
        File[] subfiles = null;
        if(files != null && files.length > 0) {
            for (File f : files) {
                if (f.isDirectory()) {
                    String id = f.getName();
                    switch (id) {
                        case "00040010":
                            mItems.add("[" + id + " - System Applications]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(getSystemApplicationName(sub.getName()));
                            }
                            break;
                        case "0004001b":
                            mItems.add("[" + id + " - System Data Archives]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(sub.getName());
                            }
                            break;
                        case "00040030":
                            mItems.add("[" + id + " - System Applets]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(sub.getName());
                            }
                            break;
                        case "0004009b":
                            mItems.add("[" + id + " - Shared Data Archives]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(getSharedDataArchiveName(sub.getName()));
                            }
                            break;
                        case "000400db":
                            mItems.add("[" + id + " - System Data Archives]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(getSystemDataArchiveName(sub.getName()));
                            }
                            break;
                        case "00040130":
                            mItems.add("[" + id + " - System Modules]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(getSystemModulesName(sub.getName()));
                            }
                            break;
                        case "00040138":
                            mItems.add("[" + id + " - System Firmware");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(sub.getName());
                            }
                            break;
                        default:
                            mItems.add("[" + id + "]");
                            subfiles = f.listFiles();
                            for (File sub : subfiles) {
                                mItems.add(sub.getName());
                            }
                            break;
                    }
                    mItems.add("");
                }
            }
        }

        mAdapter.setFileList(mItems);
    }

    protected String getSystemTitleName(String id) {
        switch (id) {
            case "00040010":
                return id + " - System Applications";
            case "0004001b":
                return id + " - System Data Archives";
            case "00040030":
                return id + " - System Applets";
            case "0004009b":
                return id + " - Shared Data Archives";
            case "000400db":
                return id + " - System Data Archives";
            case "00040130":
                return id + " - System Modules";
            case "00040138":
                return id + " - System Firmware";
            default:
                return id;
        }
    }

    protected String getSystemDataArchiveName(String id) {
        switch (id) {
            case "00010302":
                return id + " - Bad Word List";
            default:
                return id;
        }
    }

    protected String getSharedDataArchiveName(String id) {
        switch (id) {
            case "00010202":
                return id + " - Mii data";
            case "00010402":
                return id + " - Region Manifest";
            case "00014002":
                return id + " - JPN/EUR/USA System Font";
            case "00014102":
                return id + " - CHN System Font";
            case "00014202":
                return id + " - KOR System Font";
            case "00014302":
                return id + " - TWN System Font";
            default:
                return id;
        }
    }

    protected String getSystemApplicationName(String id) {
        StringBuilder sb = new StringBuilder(id);
        sb.setCharAt(4, '0');
        switch (sb.toString()) {
            case "00020000":
                return id + " - System Settings";
            case "00020100":
                return id + " - Download Play";
            case "00020400":
                return id + " - Camera";
            case "00020700":
                return id + " - Mii Maker";
            case "00020900":
                return id + " - eShop";
            default:
                return id;
        }
    }

    protected String getSystemModulesName(String id) {
        switch (id) {
            case "00001102":
                return id + " - FS";
            case "00001202":
                return id + " - PM";
            case "00003702":
                return id + " - LDR";
            case "00001402":
                return id + " - PXI";
            case "00008a02":
                return id + " - ERR";
            case "00002402":
                return id + " - AC";
            case "00003802":
                return id + " - ACT";
            case "00001502":
                return id + " - AM";
            case "00003402":
                return id + " - BOSS";
            case "00001602":
                return id + " - CAM";
            case "00002602":
                return id + " - CECD";
            case "00001702":
                return id + " - CFG";
            case "00002802":
                return id + " - DLP";
            case "00001a02":
                return id + " - DSP";
            case "00003202":
                return id + " - FRD";
            case "00001c02":
                return id + " - GSP";
            case "00001d02":
                return id + " - HID";
            case "00003302":
                return id + " - IR";
            case "00002002":
                return id + " - MIC";
            case "20004102":
                return id + " - MVD";
            case "00002b02":
                return id + " - NDM";
            case "00003502":
                return id + " - NEWS";
            case "00004002":
                return id + " - NFC";
            case "00002c02":
                return id + " - NIM";
            case "00008002":
                return id + " - NS";
            case "00002d02":
                return id + " - NWM";
            case "00002202":
                return id + " - PTM";
            case "00004202":
                return id + " - QTM";
            case "00002702":
                return id + " - CSND";
            case "00002902":
                return id + " - HTTP";
            case "00002e02":
                return id + " - SOC";
            case "00002f02":
                return id + " - SSL";
            case "00003102":
                return id + " - PS";
            case "00001f02":
                return id + " - MCU";
            case "00001802":
                return id + " - CDC";
            case "00001b02":
                return id + " - GPIO";
            case "00001e02":
                return id + " - I2C";
            case "00002a02":
                return id + " - MP";
            case "00002102":
                return id + " - PDN";
            case "00002302":
                return id + " - SPI";
            default:
                return id;
        }
    }

    protected File getNandTitlePath() {
        return new File(CitraDirectory.getSystemTitleDirectory());
    }

    static class FileViewHolder extends RecyclerView.ViewHolder {
        private String[] mInfos;
        private Button mBtnDownload;
        private TextView mTextTitle;

        public FileViewHolder(View itemView) {
            super(itemView);
            mTextTitle = itemView.findViewById(R.id.text_view);
            mBtnDownload = itemView.findViewById(R.id.btn_download);
            mBtnDownload.setVisibility(View.GONE);
            mBtnDownload.setOnClickListener( view -> {
                Log.v("citra", "Download: " + Arrays.toString(mInfos));
            });
        }

        public void bind(String info) {
            mInfos = info.split(",");
            mTextTitle.setText(mInfos[0]);
        }
    }

    static class FileAdapter extends RecyclerView.Adapter<FileViewHolder>
            implements View.OnClickListener, View.OnLongClickListener {

        private List<String> mFileList = new ArrayList<>();

        @NonNull
        @Override
        public FileViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            View view =
                    LayoutInflater.from(parent.getContext()).inflate(viewType, parent, false);
            view.setOnClickListener(this);
            view.setOnLongClickListener(this);
            return new FileViewHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull FileViewHolder holder, int position) {
            holder.bind(mFileList.get(position));
        }

        @Override
        public int getItemCount() {
            return mFileList.size();
        }

        @Override
        public int getItemViewType(int position) {
            return R.layout.file_list_item;
        }

        @Override
        public void onClick(View v) {

        }

        @Override
        public boolean onLongClick(View v) {
            return false;
        }

        public void setFileList(List<String> files) {
            mFileList = files;
            notifyDataSetChanged();
        }
    }
}
