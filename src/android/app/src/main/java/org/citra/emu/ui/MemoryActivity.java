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
import com.nononsenseapps.filepicker.DividerItemDecoration;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;

import org.citra.emu.NativeLibrary;
import org.citra.emu.R;

import java.util.ArrayList;
import java.util.List;

public final class MemoryActivity extends AppCompatActivity {
    private static final int PAGE_SIZE = 0x1000;
    private static final int PAGE_MASK = PAGE_SIZE - 1;
    private static final int PAGE_BITS = 12;
    private static final int PAGE_TABLE_NUM_ENTRIES = 1 << (32 - PAGE_BITS);

    private static int MEM_REGION_ALL_MEMORY = 0;
    private int mMemRegion = 0;

    private static int VALUE_TYPE_FOUR_BYTES = 0;
    private static int VALUE_TYPE_TWO_BYTES = 1;
    private static int VALUE_TYPE_ONE_BYTE = 2;
    private int mValueType = 0;

    private static int SEARCH_TYPE_SPECIFIED_VALUE = 0;
    private static int SEARCH_TYPE_UNKNOWN_SEARCH = 1;
    private int mSearchType = 0;

    private static int SCAN_TYPE_EQUAL_TO = 0;
    private static int SCAN_TYPE_NOT_EQUAL_TO = 1;
    private static int SCAN_TYPE_BIGGER_THAN = 2;
    private static int SCAN_TYPE_BIGGER_OR_EQUAL = 3;
    private static int SCAN_TYPE_SMALLER_THAN = 4;
    private static int SCAN_TYPE_SMALLER_OR_EQUAL = 5;
    private int mScanType = 0;

    private MemoryAdapter mAdapter;
    private RecyclerView mListView;

    private EditText mEditRegionStart;
    private EditText mEditRegionStop;
    private EditText mEditSearchValue;

    interface ListDataSet {
        int size();
        String get(int position);
    }

    public static class PageTableDataSet implements ListDataSet {
        List<String> mTexts = new ArrayList<>();

        public PageTableDataSet() {
            int[] pages = NativeLibrary.loadPageTable();
            for (int i = 0; i < pages.length; i += 2) {
                long addr = GetUnsigned(pages[i]);
                long size = GetUnsigned(pages[i + 1]);
                mTexts.add("[" + Long2Hex(addr) + ", " + Long2Hex(addr + size) + ") - " + Size2String(size));
            }
        }

        @Override
        public int size() {
            return mTexts.size();
        }

        @Override
        public String get(int position) {
            return mTexts.get(position);
        }
    }

    public static class MemoryViewDataSet implements ListDataSet {
        private final int mByteCount = 8;
        private Object[] mPointers = new Object[PAGE_TABLE_NUM_ENTRIES];
        private int[] mPageTable;
        private long mAddrStart;
        private long mAddrStop;
        private long mCount;

        public MemoryViewDataSet(long start, long stop) {
            mAddrStart = start;
            mAddrStop = stop;
            mCount = 0;
            mPageTable = NativeLibrary.loadPageTable();

            for (int i = 0; i < mPageTable.length; i += 2) {
                long addr = GetUnsigned(mPageTable[i]);
                long size = GetUnsigned(mPageTable[i + 1]);
                if (start >= addr && start < addr + size) {
                    if (stop > addr + size) {
                        mCount += (addr + size) - start;
                    } else {
                        mCount = stop - start;
                        break;
                    }
                } else if (stop < addr + size) {
                    mCount += stop - addr;
                    break;
                } else {
                    mCount += size;
                }
            }

            if (mCount < mByteCount) {
                mCount = mByteCount;
            }
        }

        @Override
        public int size() {
            return (int)(mCount / mByteCount);
        }

        @Override
        public String get(int position) {
            long targetAddr = mPageTable != null && mPageTable.length > 0 ? mPageTable[0] : 0;
            long targetOffset = position * mByteCount;
            for (int i = 0; i < mPageTable.length; ++i) {
                long addr = GetUnsigned(mPageTable[i]);
                long size = GetUnsigned(mPageTable[i + 1]);
                if (mAddrStart >= addr + size) {
                    // continue;
                } else if (mAddrStart >= addr && mAddrStart < addr + size) {
                    if (mAddrStart + targetOffset > addr + size) {
                        targetOffset -= (addr + size) - mAddrStart;
                    } else {
                        targetAddr = mAddrStart + targetOffset;
                        break;
                    }
                } else if (mAddrStart + targetOffset < addr + size) {
                    targetAddr = addr + targetOffset;
                    break;
                } else {
                    targetOffset -= size;
                }
            }

            long value0 = read32(targetAddr);
            long value1 = read32(targetAddr + 4);
            return "[" + Long2Hex(targetAddr) + "]:  " + Long2View(value0) + "  " + Long2View(value1);
        }

        public long read32(long addr) {
            int pageIndex = (int)(addr >> PAGE_BITS);
            int pageOffset = (int)(addr & PAGE_MASK);
            if (mPointers[pageIndex] == null) {
                mPointers[pageIndex] = NativeLibrary.loadPage(pageIndex);
                if (mPointers[pageIndex] == null) {
                    Log.i("zhangwei", "load page error: " + addr);
                    return 0;
                }
            }
            return GetUnsigned( ((int[])mPointers[pageIndex])[pageOffset >> 2] );
        }
    }

    public class MemoryViewHolder extends RecyclerView.ViewHolder {

        private TextView mTextView;

        public MemoryViewHolder(View itemView) {
            super(itemView);
            mTextView = itemView.findViewById(R.id.text_view);
        }

        public void bind(String text) {
            mTextView.setText(text);
        }
    }

    public class MemoryAdapter extends RecyclerView.Adapter<MemoryViewHolder> {

        private ListDataSet mDataSet;

        @NonNull
        @Override
        public MemoryViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            LayoutInflater inflater = LayoutInflater.from(parent.getContext());
            View view = inflater.inflate(R.layout.memory_list_item, parent, false);
            return new MemoryViewHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull MemoryViewHolder holder, int position) {
            holder.bind(mDataSet.get(position));
        }

        @Override
        public int getItemCount() {
            return mDataSet != null ? mDataSet.size() : 0;
        }

        public void loadDataSet(ListDataSet dataset) {
            mDataSet = dataset;
            notifyDataSetChanged();
        }
    }

    public static void launch(Context context) {
        Intent intent = new Intent(context, MemoryActivity.class);
        context.startActivity(intent);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_memory);

        Toolbar toolbar = findViewById(R.id.toolbar_main);
        setSupportActionBar(toolbar);

        mAdapter = new MemoryAdapter();
        mListView = findViewById(R.id.memory_list);
        mListView.setAdapter(mAdapter);
        Drawable lineDivider = getDrawable(R.drawable.line_divider);
        mListView.addItemDecoration(new DividerItemDecoration(lineDivider));
        RecyclerView.LayoutManager layoutManager = new LinearLayoutManager(this);
        mListView.setLayoutManager(layoutManager);
        loadPageTable();


        mEditSearchValue = findViewById(R.id.edit_search_value);
        mEditRegionStart = findViewById(R.id.edit_region_start);
        mEditRegionStop = findViewById(R.id.edit_region_stop);

        String[] memRegions = getMemRegions();
        Spinner spinnerMemRegion = findViewById(R.id.spinner_mem_region);
        ArrayAdapter<String> adapter0 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, memRegions);
        adapter0.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerMemRegion.setAdapter(adapter0);
        spinnerMemRegion.setSelection(mMemRegion);
        spinnerMemRegion.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mMemRegion = position;
                if (position == MEM_REGION_ALL_MEMORY) {
                    mEditRegionStart.setText("00000000");
                    mEditRegionStop.setText("FFFFFFFF");
                } else {
                    String item = adapter0.getItem(position);
                    int sep = item.indexOf('-');
                    mEditRegionStart.setText(item.substring(2, sep));
                    mEditRegionStop.setText(item.substring(sep + 3));
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        String[] valueTypes = getResources().getStringArray(R.array.memory_value_type);
        Spinner spinnerValueType = findViewById(R.id.spinner_value_type);
        ArrayAdapter<String> adapter1 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, valueTypes);
        adapter1.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerValueType.setAdapter(adapter1);
        spinnerValueType.setSelection(mValueType);
        spinnerValueType.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mValueType = position;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        String[] searchTypes = getResources().getStringArray(R.array.memory_search_type);
        Spinner spinnerSearchType = findViewById(R.id.spinner_search_type);
        ArrayAdapter<String> adapter2 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, searchTypes);
        adapter2.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerSearchType.setAdapter(adapter2);
        spinnerSearchType.setSelection(mSearchType);
        spinnerSearchType.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mSearchType = position;
                mEditSearchValue.setEnabled(position == SEARCH_TYPE_SPECIFIED_VALUE);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        String[] scanTypes = getResources().getStringArray(R.array.memory_scan_type);
        Spinner spinnerScanType = findViewById(R.id.spinner_scan_type);
        ArrayAdapter<String> adapter3 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, scanTypes);
        adapter3.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerScanType.setAdapter(adapter3);
        spinnerScanType.setSelection(mScanType);
        spinnerScanType.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mScanType = position;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        Button btnToHex = findViewById(R.id.btn_to_hex);
        btnToHex.setOnClickListener(view -> {
            String strValue = mEditSearchValue.getText().toString();
            if (strValue.matches(".*[a-zA-Z]+.*") || strValue.length() >= 8) {
                // nothing to do
            }  else if (strValue.contains(".")) {
                // float to hex
                float value = 0.0f;
                try {
                    value = Float.parseFloat(strValue);
                } catch (NumberFormatException e) {
                    // ignore
                }
                String text = Integer.toHexString(Float.floatToIntBits(value)).toUpperCase();
                mEditSearchValue.setText(text);
                mEditSearchValue.setSelection(text.length());
            } else {
                // integer to hex
                int value = 0;
                try {
                    value = Integer.parseInt(strValue);
                } catch (NumberFormatException e) {
                    // ignore
                }
                String text = Integer.toHexString(value).toUpperCase();
                mEditSearchValue.setText(text);
                mEditSearchValue.setSelection(text.length());
            }
        });

        Button btnSearch = findViewById(R.id.btn_search);
        btnSearch.setOnClickListener(view -> {
            String strValue = mEditSearchValue.getText().toString();
            int value = 0;
            try {
                value = Integer.parseInt(strValue);
            } catch (NumberFormatException e) {
                // ignore
            }
            NativeLibrary.searchMemoryRegion(mMemRegion, mValueType, mSearchType, mScanType, value);
        });

        Button btnUndo = findViewById(R.id.btn_undo);
        btnUndo.setOnClickListener(view -> {
            String strStart = mEditRegionStart.getText().toString();
            String strStop = mEditRegionStop.getText().toString();
            long start = Hex2Long(strStart);
            long stop = Hex2Long(strStop);
            loadMemory(start, stop);
        });

        Button btnReset = findViewById(R.id.btn_reset);
        btnReset.setOnClickListener(view -> {
            loadPageTable();
        });
    }

    public void loadPageTable() {
        mAdapter.loadDataSet(new PageTableDataSet());
        mListView.scrollToPosition(0);
    }

    public void loadMemory(long start, long stop) {
        if (stop > start) {
            mAdapter.loadDataSet(new MemoryViewDataSet(start, stop));
            mListView.scrollToPosition(0);
        }
    }

    public String[] getMemRegions() {
        List<String> regions = new ArrayList<>();
        int[] pages = NativeLibrary.loadPageTable();

        regions.add(getString(R.string.memory_search_all_memory));
        for (int i = 0; i < pages.length; i += 2) {
            long addr = GetUnsigned(pages[i]);
            long size = GetUnsigned(pages[i + 1]);
            regions.add(Long2Hex(addr) + "-" + Long2Hex(addr + size));
        }

        return regions.toArray(new String[0]);
    }

    public static String Size2String(long size) {
        final int KB = 1024;
        final int MB = KB * KB;
        final int GB = KB * MB;

        StringBuilder sb = new StringBuilder();

        if (size > GB) {
            sb.append(size / GB);
            sb.append(" GiB ");
            size %= GB;
        }

        if (size > MB) {
            sb.append(size / MB);
            sb.append(" MiB ");
            size %= MB;
        }

        if (size > KB) {
            sb.append(size / KB);
            sb.append(" KiB ");
            size %= KB;
        }

        if (size > 0) {
            sb.append(size);
            sb.append(" B");
        }

        return sb.toString();
    }

    public static String Long2View(long value) {
        char[] hexChars = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
        StringBuilder sb = new StringBuilder();
        int gap = 0;
        for (int i = 0; i < 8; ++i) {
            sb.insert(0, hexChars[(int)(value & 0xF)]);
            value >>= 4;
            if ((i & 0x1) == 1) {
                sb.insert(0, ' ');
            }
        }
        return sb.toString();
    }

    public static String Long2Hex(long value) {
        char[] hexChars = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 8; ++i) {
            sb.insert(0, hexChars[(int)(value & 0xF)]);
            value >>= 4;
        }
        sb.insert(0, "0x");
        return sb.toString();
    }

    public static long Hex2Long(String text) {
        long value = 0;
        for (int i = 0; i < text.length(); ++i) {
            int v = Character.digit(text.charAt(i), 16);
            if (v != -1) {
                value = value << 4 | v;
            } else {
                break;
            }
        }
        return value;
    }

    public static long GetUnsigned(int signed) {
        return signed >= 0 ? signed : 2 * (long) Integer.MAX_VALUE + 2 + signed;
    }
}
