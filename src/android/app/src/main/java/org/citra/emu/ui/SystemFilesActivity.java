// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.content.Context;
import android.content.Intent;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.GridLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;

import com.nononsenseapps.filepicker.DividerItemDecoration;

import org.citra.emu.R;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public final class SystemFilesActivity extends AppCompatActivity {

    public static void launch(Context context) {
        Intent settings = new Intent(context, SystemFilesActivity.class);
        context.startActivity(settings);
    }

    private ProgressBar mProgressBar;
    private RecyclerView mListView;
    private FileAdapter mAdapter;

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
        int columns = getResources().getInteger(R.integer.game_grid_columns);
        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        mListView.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new GridLayoutManager(this, columns);
        mListView.setLayoutManager(layoutManager);

        loadSystemFiles();
    }

    protected void loadSystemFiles() {
        List<String> files = new ArrayList<>();
        loadStringArray(files, R.array.system_firmwares);
        loadStringArray(files, R.array.system_applications);
        mAdapter.setFileList(files);
    }

    protected void loadStringArray(List<String> files, int resId) {
        String[] array = getResources().getStringArray(resId);
        Collections.addAll(files, array);
    }

    static class FileViewHolder extends RecyclerView.ViewHolder {
        private String[] mInfos;
        private Button mBtnDownload;
        private TextView mTextTitle;

        public FileViewHolder(View itemView) {
            super(itemView);
            mTextTitle = itemView.findViewById(R.id.text_view);
            mBtnDownload = itemView.findViewById(R.id.btn_download);
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
