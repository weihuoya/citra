// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.DialogFragment;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.nononsenseapps.filepicker.DividerItemDecoration;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;
import org.citra.emu.utils.DirectoryInitialization;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public final class AmiiboDialog extends DialogFragment {

    public static AmiiboDialog newInstance() {
        return new AmiiboDialog();
    }

    @NonNull
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());

        ViewGroup contents =
            (ViewGroup)getActivity().getLayoutInflater().inflate(R.layout.dialog_amiibo, null);

        TextView title = contents.findViewById(R.id.text_title);
        title.setText("Amiibo");

        Drawable lineDivider = getContext().getDrawable(R.drawable.line_divider);
        RecyclerView recyclerView = contents.findViewById(R.id.list_settings);
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(getContext());
        recyclerView.setLayoutManager(layoutManager);
        FileAdapter adapter = new FileAdapter();
        recyclerView.setAdapter(adapter);
        recyclerView.addItemDecoration(new DividerItemDecoration(lineDivider));

        // load amiibo list
        List<String> amiibos = new ArrayList<>();
        String amiiboDir = DirectoryInitialization.getAmiiboDirectory();
        File[] files = new File(amiiboDir).listFiles((File dir, String name) -> name.endsWith(".bin"));
        for(File f : files) {
            amiibos.add(f.getPath());
        }
        adapter.setFileList(amiibos);

        TextView textHint = contents.findViewById(R.id.text_amiibo_hint);
        textHint.setVisibility(amiibos.size() > 0 ? View.INVISIBLE : View.VISIBLE);

        builder.setView(contents);
        return builder.create();
    }

    static class FileViewHolder extends RecyclerView.ViewHolder {
        private String mPath;
        private TextView mTextTitle;

        public FileViewHolder(View itemView) {
            super(itemView);
            itemView.setTag(this);
            mTextTitle = itemView.findViewById(R.id.text_view);
        }

        public String getPath() {
            return mPath;
        }

        public void bind(String path) {
            String filename = path.substring(path.lastIndexOf(File.separatorChar) + 1, path.length() - 4);
            mTextTitle.setText(filename);
            mPath = path;
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
            return R.layout.amiibo_list_item;
        }

        @Override
        public void onClick(View v) {
            FileViewHolder holder = (FileViewHolder)v.getTag();
            String path = holder.getPath();
            NativeLibrary.loadAmiibo(path);
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
