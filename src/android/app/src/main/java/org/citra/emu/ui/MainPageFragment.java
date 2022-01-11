package org.citra.emu.ui;

import android.app.Activity;
import android.content.Context;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.recyclerview.widget.DiffUtil;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import com.nononsenseapps.filepicker.DividerItemDecoration;

import org.citra.emu.R;
import org.citra.emu.model.GameFile;
import org.citra.emu.utils.CitraDirectory;
import org.citra.emu.utils.FileBrowserHelper;

import java.io.File;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class MainPageFragment extends Fragment {

    private static final String ARG_POSITION = "ARG_POSITION";

    private int mPosition;
    private GameListAdapter mAdapter;
    private TextView mPageInfo;
    private Button mBtnAction;
    private RecyclerView mRecyclerView;

    public static MainPageFragment newInstance(int position) {
        MainPageFragment fragment = new MainPageFragment();
        Bundle args = new Bundle();
        args.putInt(ARG_POSITION, position);
        fragment.setArguments(args);
        return fragment;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mPosition = getArguments().getInt(ARG_POSITION);
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.fragment_main_page, container, false);

        mPageInfo = root.findViewById(R.id.page_info);
        mBtnAction = root.findViewById(R.id.page_action);
        mRecyclerView = root.findViewById(R.id.page_grid);

        mAdapter = new GameListAdapter();
        mRecyclerView.setAdapter(mAdapter);
        Context context = requireContext();
        int columns = context.getResources().getInteger(R.integer.game_grid_columns);
        Drawable lineDivider = context.getDrawable(R.drawable.line_divider);
        mRecyclerView.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new GridLayoutManager(context, columns);
        mRecyclerView.setLayoutManager(layoutManager);

        if (mPosition == 0) {
            mPageInfo.setText(R.string.game_emulation_info);
            mBtnAction.setText(R.string.add_directory_title);
            mBtnAction.setOnClickListener(view -> FileBrowserHelper.openDirectoryPicker((Activity) view.getContext()));
        } else {
            mPageInfo.setText(R.string.application_emulation_info);
            mBtnAction.setText(R.string.grid_menu_install_cia);
            mBtnAction.setOnClickListener(view -> FileBrowserHelper.openFilePicker((Activity) view.getContext()));
        }

        return root;
    }

    @Override
    public void onConfigurationChanged(@NonNull Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
    }

    public void setGames(List<GameFile> games) {
        if (games.size() == 0) {
            mPageInfo.setVisibility(View.VISIBLE);
            mBtnAction.setVisibility(View.VISIBLE);
            mRecyclerView.setVisibility(View.INVISIBLE);
        } else {
            mPageInfo.setVisibility(View.INVISIBLE);
            mBtnAction.setVisibility(View.INVISIBLE);
            mRecyclerView.setVisibility(View.VISIBLE);
        }
        mAdapter.setGames(games);
    }

    private static class GameViewHolder extends RecyclerView.ViewHolder {
        private final ImageView mImageIcon;
        private final TextView mTextTitle;
        private final TextView mTextRegion;
        private final TextView mTextCompany;
        private final DecimalFormat mFormat;
        private GameFile mModel;

        public GameViewHolder(View itemView) {
            super(itemView);
            itemView.setTag(this);
            mImageIcon = itemView.findViewById(R.id.image_game_screen);
            mTextTitle = itemView.findViewById(R.id.text_game_title);
            mTextRegion = itemView.findViewById(R.id.text_region);
            mTextCompany = itemView.findViewById(R.id.text_company);
            mFormat = new DecimalFormat("#.##");
        }

        public void bind(GameFile model) {
            mModel = model;
            mTextTitle.setText(model.getName());
            if ("0004000000000000".equals(model.getId())) {
                mTextTitle.setTextColor(Color.RED);
            } else {
                mTextTitle.setTextColor(Color.BLACK);
            }
            mTextCompany.setText(model.getInfo());
            mTextRegion.setText(getAppDesc(model));
            mImageIcon.setImageBitmap(model.getIcon());
        }

        public String getAppDesc(GameFile model) {
            final int[] regions = {
                    R.string.region_invalid, R.string.region_japan,     R.string.region_north_america,
                    R.string.region_europe,  R.string.region_australia, R.string.region_china,
                    R.string.region_korea,   R.string.region_taiwan,
            };
            String path = model.getPath();
            File appFile = new File(path);

            long playtime = 0;
            Context context = mTextRegion.getContext();
            String regionDesc = context.getString(regions[model.getRegion() + 1]);
            String desc = "";

            if (model.isInstalledDLC()) {
                desc = context.getString(R.string.installed_app, Size2String(appFile.length()));
                desc += " DLC";
            } else if (playtime >= 2 * 3600) {
                String time = mFormat.format(playtime / 3600.0f);
                desc = context.getString(R.string.playtime_hours, time);
                desc = context.getString(R.string.playtime_desc, desc);
            } else if (playtime >= 60) {
                String time = mFormat.format(playtime / 60.0f);
                desc = context.getString(R.string.playtime_minutes, time);
                desc = context.getString(R.string.playtime_desc, desc);
            }

            StringBuilder sb = new StringBuilder();
            sb.append(regionDesc);
            if (!desc.isEmpty()) {
                sb.append(" (");
                sb.append(desc);
                sb.append(")");
            }
            return sb.toString();
        }

        private String Size2String(long size) {
            final int KB = 1024;
            final int MB = KB * KB;
            final int GB = KB * MB;
            StringBuilder sb = new StringBuilder();
            if (size > GB) {
                sb.append(mFormat.format(size / (float)GB));
                sb.append("GB");
            } else if (size > MB) {
                sb.append(mFormat.format(size / (float)MB));
                sb.append("MB");
            } else if (size > KB) {
                sb.append(mFormat.format(size / (float)KB));
                sb.append("KB");
            } else if (size > 0) {
                sb.append(size);
                sb.append("B");
            }
            return sb.toString();
        }

        public GameFile getModel() {
            return mModel;
        }
    }

    private static class GameListAdapter extends RecyclerView.Adapter<GameViewHolder>
            implements View.OnClickListener, View.OnLongClickListener {
        private List<GameFile> mGames = new ArrayList<>();

        @Override
        public GameViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View gameCard =
                    LayoutInflater.from(parent.getContext()).inflate(viewType, parent, false);
            gameCard.setOnClickListener(this);
            gameCard.setOnLongClickListener(this);
            return new GameViewHolder(gameCard);
        }

        @Override
        public void onBindViewHolder(GameViewHolder holder, int position) {
            holder.bind(mGames.get(position));
        }

        @Override
        public int getItemViewType(int position) {
            return R.layout.card_game;
        }

        @Override
        public int getItemCount() {
            return mGames.size();
        }

        @Override
        public void onClick(View view) {
            GameViewHolder holder = (GameViewHolder)view.getTag();
            GameFile model = holder.getModel();
            if (CitraDirectory.isInitialized() && !model.isInstalledDLC()) {
                EmulationActivity.launch(view.getContext(), model);
            }
        }

        @Override
        public boolean onLongClick(View view) {
            GameViewHolder holder = (GameViewHolder)view.getTag();
            EditorActivity.launch(view.getContext(), holder.getModel());
            return true;
        }

        public void setGames(List<GameFile> games) {
            if (mGames.size() == 0) {
                if (games.size() > 0) {
                    mGames = games;
                    notifyDataSetChanged();
                }
                return;
            }

            DiffUtil.DiffResult result = DiffUtil.calculateDiff(new DiffUtil.Callback() {
                @Override
                public int getOldListSize() {
                    return mGames.size();
                }

                @Override
                public int getNewListSize() {
                    return games.size();
                }

                @Override
                public boolean areItemsTheSame(int oldItemPosition, int newItemPosition) {
                    GameFile newGame = games.get(newItemPosition);
                    GameFile oldGame = mGames.get(oldItemPosition);
                    return newGame.getId().equals(oldGame.getId());
                }

                @Override
                public boolean areContentsTheSame(int oldItemPosition, int newItemPosition) {
                    GameFile newGame = games.get(newItemPosition);
                    GameFile oldGame = mGames.get(oldItemPosition);
                    if (!newGame.getId().equals(oldGame.getId())) {
                        return false;
                    } else {
                        Bitmap newIcon = newGame.getIcon();
                        Bitmap oldIcon = oldGame.getIcon();
                        return newIcon.sameAs(oldIcon);
                    }
                }
            });

            mGames = games;
            result.dispatchUpdatesTo(this);
        }
    }
}
