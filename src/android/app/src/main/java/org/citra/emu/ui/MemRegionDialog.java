package org.citra.emu.ui;


import android.app.AlertDialog;
import android.app.Dialog;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.DialogFragment;
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

public class MemRegionDialog extends DialogFragment {

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

    private boolean mIsHexValue = false;

    private EditText mEditRegionStart;
    private EditText mEditRegionStop;
    private EditText mEditSearchValue;

    public static MemRegionDialog newInstance() {
        MemRegionDialog fragment = new MemRegionDialog();
        return fragment;
    }

    @NonNull
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        ViewGroup contents =
                (ViewGroup)getActivity().getLayoutInflater().inflate(R.layout.dialog_mem_region, null);

        mEditSearchValue = contents.findViewById(R.id.edit_search_value);
        mEditRegionStart = contents.findViewById(R.id.edit_region_start);
        mEditRegionStop = contents.findViewById(R.id.edit_region_stop);

        String[] memRegions = {"All memory"};
        Spinner spinnerMemRegion = contents.findViewById(R.id.spinner_mem_region);
        ArrayAdapter<String> adapter0 = new ArrayAdapter<>(getContext(), android.R.layout.simple_spinner_item, memRegions);
        adapter0.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerMemRegion.setAdapter(adapter0);
        spinnerMemRegion.setSelection(mMemRegion);
        spinnerMemRegion.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                mMemRegion = position;
                if (position == MEM_REGION_ALL_MEMORY) {
                    mEditRegionStart.setText("00000000");
                    mEditRegionStop.setText("FFFFFFF0");
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        String[] valueTypes = {"4 Bytes", "2 Bytes", "1 Bytes"};
        Spinner spinnerValueType = contents.findViewById(R.id.spinner_value_type);
        ArrayAdapter<String> adapter1 = new ArrayAdapter<>(getContext(), android.R.layout.simple_spinner_item, valueTypes);
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

        String[] searchTypes = {"Specified value", "Unknown search"};
        Spinner spinnerSearchType = contents.findViewById(R.id.spinner_search_type);
        ArrayAdapter<String> adapter2 = new ArrayAdapter<>(getContext(), android.R.layout.simple_spinner_item, searchTypes);
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

        String[] scanTypes = {"Equal To", "Not Equal To", "Bigger Than", "Bigger Or Equal", "Smaller Than", "Smaller Or Equal"};
        Spinner spinnerScanType = contents.findViewById(R.id.spinner_scan_type);
        ArrayAdapter<String> adapter3 = new ArrayAdapter<>(getContext(), android.R.layout.simple_spinner_item, scanTypes);
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

        Button btnToHex = contents.findViewById(R.id.btn_to_hex);
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

        Button btnSearch = contents.findViewById(R.id.btn_search);
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

        builder.setView(contents);
        return builder.create();
    }
}
