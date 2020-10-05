package com.hri_physio.polarstreamer;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;

import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

public class PolarOH1Frag extends Fragment {
    private String DEVICE_ID;
    public SharedPreferences sharedPreferences;
    private String sharedPrefsKey = "polar_device_id";

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.polar_oh1_frag, container, false);
        sharedPreferences = this.getActivity().getPreferences(Context.MODE_PRIVATE);

        EditText enterIdText = (EditText) view.findViewById(R.id.buttonSetID);
        enterIdText.setInputType(InputType.TYPE_CLASS_TEXT);
        enterIdText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {  }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {  }

            @Override
            public void afterTextChanged(Editable s) {
                DEVICE_ID = enterIdText.getText().toString();
                SharedPreferences.Editor editor = sharedPreferences.edit();
                editor.putString(sharedPrefsKey, DEVICE_ID);
            }
        });
        return view;
    }
}
