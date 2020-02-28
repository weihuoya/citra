// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.annotation.NonNull;
import android.support.v4.app.DialogFragment;
import android.support.v7.app.AppCompatActivity;
import com.nononsenseapps.filepicker.DividerItemDecoration;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.support.v7.widget.Toolbar;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.RadioGroup;
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

    //private static final String PREF_MEM_REGION = "search_mem_region";
    private static final String PREF_VALUE_TYPE = "search_value_type";
    private static final String PREF_SEARCH_TYPE = "search_type";
    private static final String PREF_SCAN_TYPE = "search_scan_type";
    private static final String PREF_ADDR_START = "search_addr_start";
    private static final String PREF_ADDR_STOP = "search_addr_stop";
    private static final String RPEF_VALUE_IS_HEX = "search_value_is_hex";
    private static final String PREF_SEARCH_VALUE = "search_value";

    private MemoryAdapter mAdapter;
    private RecyclerView mListView;

    private Spinner mSpinnerMemRegion;
    private Spinner mSpinnerValueType;
    private Spinner mSpinnerSearchType;
    private Spinner mSpinnerScanType;

    private EditText mEditRegionStart;
    private EditText mEditRegionStop;
    private EditText mEditSearchValue;
    private CheckBox mHexCheckBox;

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
                mTexts.add("[" + Long2Hex(addr) + ", " + Long2Hex(addr + size) + ") - " + NativeLibrary.Size2String(size));
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

    public static class SearchResultDataSet implements ListDataSet {
        private int[] mResults;

        public SearchResultDataSet(int[] results) {
            mResults = results;
        }

        @Override
        public int size() {
            return (mResults.length / 2) + 1;
        }

        @Override
        public String get(int position) {
            if (position == 0) {
                return String.format("Search results: %d (Click to edit)", mResults.length / 2);
            } else {
                int i = (position - 1) * 2;
                long addr = GetUnsigned(mResults[i]);
                long value = GetUnsigned(mResults[i + 1]);
                return "[" + Long2Hex(addr) + "]:  " + Long2Hex(value);
            }
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
                    // load page error
                    return 0;
                }
            }
            return Bytes2Long((byte[]) mPointers[pageIndex], pageOffset, 4);
        }
    }

    public class MemoryHolder extends RecyclerView.ViewHolder {

        private TextView mTextView;

        public MemoryHolder(View itemView) {
            super(itemView);
            mTextView = itemView.findViewById(R.id.text_view);
            itemView.setOnClickListener((View v) -> {
                String text = mTextView.getText().toString();
                if (text.isEmpty()) {
                    return;
                }
                String textAddr = text.substring(3, 11);
                int addr = GetSigned(Hex2Long(textAddr));
                MemoryEditorDialog.newInstance(addr).show(getSupportFragmentManager(), "MemoryEditorDialog");
            });
        }

        public void bind(String text) {
            mTextView.setText(text);
        }
    }

    public class MemoryAdapter extends RecyclerView.Adapter<MemoryHolder> {

        private ListDataSet mDataSet;

        @NonNull
        @Override
        public MemoryHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            LayoutInflater inflater = LayoutInflater.from(parent.getContext());
            View view = inflater.inflate(R.layout.memory_list_item, parent, false);
            return new MemoryHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull MemoryHolder holder, int position) {
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

    public static class MemoryEditorDialog extends DialogFragment {

        private static final String ARG_MEM_ADDR = "address";

        public static MemoryEditorDialog newInstance(int address) {
            MemoryEditorDialog fragment = new MemoryEditorDialog();
            Bundle arguments = new Bundle();
            arguments.putInt(ARG_MEM_ADDR, address);
            fragment.setArguments(arguments);
            return fragment;
        }

        private int mValueType = VALUE_TYPE_FOUR_BYTES;

        @NonNull
        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            Bundle args = getArguments();
            int addr = args.getInt(ARG_MEM_ADDR, 0);
            AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
            ViewGroup contents =
                    (ViewGroup)getActivity().getLayoutInflater().inflate(R.layout.dialog_memory_editor, null);

            EditText editAddr = contents.findViewById(R.id.mem_region);
            editAddr.setSelectAllOnFocus(true);
            editAddr.setText(Long2Hex(GetUnsigned(addr)).substring(2));

            EditText editHex = contents.findViewById(R.id.edit_hex_value);
            EditText editInt = contents.findViewById(R.id.edit_integer_value);
            EditText editFloat = contents.findViewById(R.id.edit_float_value);

            // memory value
            int memValue = NativeLibrary.readMemory(addr, mValueType);
            editHex.setText(Long2Hex(GetUnsigned(memValue)).substring(2));
            editInt.setText(Long.toString(GetUnsigned(memValue)));
            editFloat.setText(Float.toString(Float.intBitsToFloat(memValue)));

            editHex.addTextChangedListener(new TextWatcher() {
                @Override
                public void beforeTextChanged(CharSequence s, int start, int count, int after) {

                }

                @Override
                public void onTextChanged(CharSequence s, int start, int before, int count) {

                }

                @Override
                public void afterTextChanged(Editable s) {
                    if (!editHex.isFocused()) {
                        return;
                    }
                    String text = s.toString();
                    if (text.isEmpty()) {
                        editInt.setText(text);
                        editFloat.setText(text);
                    } else {
                        long value = Hex2Long(text);
                        editInt.setText(Long.toString(value));
                        float f = Float.intBitsToFloat(GetSigned(value));
                        editFloat.setText(Float.toString(f));
                    }
                }
            });
            editInt.addTextChangedListener(new TextWatcher() {
                @Override
                public void beforeTextChanged(CharSequence s, int start, int count, int after) {

                }

                @Override
                public void onTextChanged(CharSequence s, int start, int before, int count) {

                }

                @Override
                public void afterTextChanged(Editable s) {
                    if (!editInt.isFocused()) {
                        return;
                    }
                    String text = s.toString();
                    if (text.isEmpty()) {
                        editHex.setText(text);
                        editFloat.setText(text);
                    } else {
                        long value;
                        try {
                            value = Long.valueOf(text);
                        } catch (Exception e) {
                            value = 0;
                        }
                        editHex.setText(Long2Hex(value).substring(2));
                        float f = Float.intBitsToFloat(GetSigned(value));
                        editFloat.setText(Float.toString(f));
                    }
                }
            });
            editFloat.addTextChangedListener(new TextWatcher() {
                @Override
                public void beforeTextChanged(CharSequence s, int start, int count, int after) {

                }

                @Override
                public void onTextChanged(CharSequence s, int start, int before, int count) {

                }

                @Override
                public void afterTextChanged(Editable s) {
                    if (!editFloat.isFocused()) {
                        return;
                    }
                    String text = s.toString();
                    if (text.isEmpty()) {
                        editHex.setText(text);
                        editInt.setText(text);
                    } else {
                        float f;
                        try {
                            f = Float.valueOf(text);
                        } catch (Exception e) {
                            f = 0.0f;
                        }
                        long value = GetUnsigned(Float.floatToIntBits(f));
                        editHex.setText(Long2Hex((value)).substring(2));
                        editInt.setText(Long.toString(value));
                    }
                }
            });

            Button btnNext = contents.findViewById(R.id.mem_region_next);
            btnNext.setOnClickListener((View v) -> {
                String text = editAddr.getText().toString();
                long value = Hex2Long(text);
                if (mValueType == VALUE_TYPE_FOUR_BYTES) {
                    value += 4;
                } else if (mValueType == VALUE_TYPE_TWO_BYTES) {
                    value += 2;
                } else if (mValueType == VALUE_TYPE_ONE_BYTE) {
                    value += 1;
                }
                editAddr.setText(Long2Hex(value).substring(2));

                int rawValue = NativeLibrary.readMemory(GetSigned(value), mValueType);
                editHex.setText(Long2Hex(GetUnsigned(rawValue)).substring(2));
                editInt.setText(Long.toString(GetUnsigned(rawValue)));
                editFloat.setText(Float.toString(Float.intBitsToFloat(rawValue)));
            });

            Button btnPrevious = contents.findViewById(R.id.mem_region_previous);
            btnPrevious.setOnClickListener((View v) -> {
                String text = editAddr.getText().toString();
                long value = Hex2Long(text);
                if (mValueType == VALUE_TYPE_FOUR_BYTES) {
                    value -= 4;
                } else if (mValueType == VALUE_TYPE_TWO_BYTES) {
                    value -= 2;
                } else if (mValueType == VALUE_TYPE_ONE_BYTE) {
                    value -= 1;
                }
                editAddr.setText(Long2Hex(value).substring(2));

                int rawValue = NativeLibrary.readMemory(GetSigned(value), mValueType);
                editHex.setText(Long2Hex(GetUnsigned(rawValue)).substring(2));
                editInt.setText(Long.toString(GetUnsigned(rawValue)));
                editFloat.setText(Float.toString(Float.intBitsToFloat(rawValue)));
            });

            RadioGroup valueTypes = contents.findViewById(R.id.radio_group);
            valueTypes.setOnCheckedChangeListener((RadioGroup group, int checkedId) -> {
                if (checkedId == R.id.radio0) {
                    mValueType = VALUE_TYPE_FOUR_BYTES;
                } else if (checkedId == R.id.radio1) {
                    mValueType = VALUE_TYPE_TWO_BYTES;
                } else if (checkedId == R.id.radio2) {
                    mValueType = VALUE_TYPE_ONE_BYTE;
                }
                String text = editAddr.getText().toString();
                long value = Hex2Long(text);
                int rawValue = NativeLibrary.readMemory(GetSigned(value), mValueType);
                editHex.setText(Long2Hex(GetUnsigned(rawValue)).substring(2));
                editInt.setText(Long.toString(GetUnsigned(rawValue)));
                editFloat.setText(Float.toString(Float.intBitsToFloat(rawValue)));
            });

            Button btnRead = contents.findViewById(R.id.btn_read);
            btnRead.setOnClickListener((View v) -> {
                String text = editAddr.getText().toString();
                long value = Hex2Long(text);
                int rawValue = NativeLibrary.readMemory(GetSigned(value), mValueType);
                editHex.setText(Long2Hex(GetUnsigned(rawValue)).substring(2));
                editInt.setText(Long.toString(GetUnsigned(rawValue)));
                editFloat.setText(Float.toString(Float.intBitsToFloat(rawValue)));
            });

            Button btnWrite = contents.findViewById(R.id.btn_write);
            btnWrite.setOnClickListener((View v) -> {
                String textAddr = editAddr.getText().toString();
                String textValue = editHex.getText().toString();
                long address = Hex2Long(textAddr);
                long value = Hex2Long(textValue);
                NativeLibrary.writeMemory(GetSigned(address), mValueType, GetSigned(value));
            });

            builder.setView(contents);
            return builder.create();
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

        int[] previousResults = NativeLibrary.getSearchResults();
        if (previousResults.length > 0) {
            loadResults(previousResults);
        } else {
            loadPageTable();
        }

        mEditSearchValue = findViewById(R.id.edit_search_value);
        mEditRegionStart = findViewById(R.id.edit_region_start);
        mEditRegionStop = findViewById(R.id.edit_region_stop);
        mHexCheckBox = findViewById(R.id.checkbox_hex);

        mEditSearchValue.setSelectAllOnFocus(true);
        mEditRegionStart.setSelectAllOnFocus(true);
        mEditRegionStop.setSelectAllOnFocus(true);

        String[] memRegions = getMemRegions();
        mSpinnerMemRegion = findViewById(R.id.spinner_mem_region);
        ArrayAdapter<String> adapter0 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, memRegions);
        adapter0.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mSpinnerMemRegion.setAdapter(adapter0);
        mSpinnerMemRegion.setSelection(mMemRegion);
        mSpinnerMemRegion.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
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
        mSpinnerValueType = findViewById(R.id.spinner_value_type);
        ArrayAdapter<String> adapter1 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, valueTypes);
        adapter1.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mSpinnerValueType.setAdapter(adapter1);
        mSpinnerValueType.setSelection(mValueType);
        mSpinnerValueType.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mValueType = position;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        String[] searchTypes = getResources().getStringArray(R.array.memory_search_type);
        mSpinnerSearchType = findViewById(R.id.spinner_search_type);
        ArrayAdapter<String> adapter2 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, searchTypes);
        adapter2.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mSpinnerSearchType.setAdapter(adapter2);
        mSpinnerSearchType.setSelection(mSearchType);
        mSpinnerSearchType.setEnabled(false);
        mSpinnerSearchType.setClickable(false);
        mSpinnerSearchType.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
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
        mSpinnerScanType = findViewById(R.id.spinner_scan_type);
        ArrayAdapter<String> adapter3 = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, scanTypes);
        adapter3.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mSpinnerScanType.setAdapter(adapter3);
        mSpinnerScanType.setSelection(mScanType);
        mSpinnerScanType.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mScanType = position;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        Button btnSearch = findViewById(R.id.btn_search);
        btnSearch.setOnClickListener(view -> {
            String strValue = mEditSearchValue.getText().toString();
            String strStart = mEditRegionStart.getText().toString();
            String strStop = mEditRegionStop.getText().toString();
            boolean isHex = mHexCheckBox.isChecked();
            int startAddr = GetSigned(Hex2Long(strStart));
            int stopAddr = GetSigned(Hex2Long(strStop));
            long value = 0;
            try {
                value = Long.parseLong(strValue, isHex ? 16 : 10);
            } catch (NumberFormatException e) {
                // ignore
            }
            int[] results = NativeLibrary.searchMemory(startAddr, stopAddr, mValueType, mSearchType, mScanType, GetSigned(value));
            loadResults(results);
        });

        Button btnView = findViewById(R.id.btn_view);
        btnView.setOnClickListener(view -> {
            String strStart = mEditRegionStart.getText().toString();
            String strStop = mEditRegionStop.getText().toString();
            long start = Hex2Long(strStart);
            long stop = Hex2Long(strStop);
            loadMemory(start, stop);
        });

        Button btnReset = findViewById(R.id.btn_reset);
        btnReset.setOnClickListener(view -> {
            NativeLibrary.resetSearchResults();
            loadPageTable();
        });
    }

    @Override
    protected void onPause() {
        super.onPause();

        SharedPreferences.Editor editor = PreferenceManager.getDefaultSharedPreferences(this).edit();
        editor.putInt(PREF_SEARCH_TYPE, mSearchType);
        editor.putInt(PREF_SCAN_TYPE, mScanType);
        editor.putInt(PREF_VALUE_TYPE, mValueType);
        //editor.putInt(PREF_MEM_REGION, mMemRegion);
        editor.putBoolean(RPEF_VALUE_IS_HEX, mHexCheckBox.isChecked());
        editor.putString(PREF_ADDR_START, mEditRegionStart.getText().toString());
        editor.putString(PREF_ADDR_STOP, mEditRegionStop.getText().toString());
        editor.putString(PREF_SEARCH_VALUE, mEditSearchValue.getText().toString());
        editor.apply();
    }

    @Override
    protected void onResume() {
        super.onResume();

        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
        mSearchType = pref.getInt(PREF_SEARCH_TYPE, mSearchType);
        mSpinnerSearchType.setSelection(mSearchType);
        mScanType = pref.getInt(PREF_SCAN_TYPE, mScanType);
        mSpinnerScanType.setSelection(mScanType);
        mValueType = pref.getInt(PREF_VALUE_TYPE, mValueType);
        mSpinnerValueType.setSelection(mValueType);
        //mMemRegion = pref.getInt(PREF_MEM_REGION, mMemRegion);
        //mSpinnerMemRegion.setSelection(mMemRegion);
        mHexCheckBox.setChecked(pref.getBoolean(RPEF_VALUE_IS_HEX, false));
        mEditRegionStart.setText(pref.getString(PREF_ADDR_START, mEditRegionStart.getText().toString()));
        mEditRegionStop.setText(pref.getString(PREF_ADDR_STOP, mEditRegionStop.getText().toString()));
        mEditSearchValue.setText(pref.getString(PREF_SEARCH_VALUE, mEditSearchValue.getText().toString()));
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

    public void loadResults(int[] results) {
        mAdapter.loadDataSet(new SearchResultDataSet(results));
        mListView.scrollToPosition(0);
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
        return signed < 0 ? 2 * (long) Integer.MAX_VALUE + 2 + signed : signed;
    }

    public static int GetSigned(long unsigned) {
        return (int) (unsigned > Integer.MAX_VALUE ? unsigned - 2 * (long) Integer.MAX_VALUE - 2 : unsigned);
    }

    public static long Bytes2Long(byte[] data, int offset, int size) {
        long value = 0;
        offset += size - 1;
        for(int i = 0; i < size; ++i) {
            value = (value << 8) | (data[offset - i] & 0xFF);
        }
        return value;
    }
}
