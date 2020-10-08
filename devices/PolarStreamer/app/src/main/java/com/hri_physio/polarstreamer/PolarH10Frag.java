package com.hri_physio.polarstreamer;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.os.SystemClock;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Chronometer;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.fragment.app.Fragment;
import java.util.UUID;
import polar.com.sdk.api.PolarBleApi;
import polar.com.sdk.api.PolarBleApiCallback;
import polar.com.sdk.api.PolarBleApiDefaultImpl;
import polar.com.sdk.api.errors.PolarInvalidArgument;
import polar.com.sdk.api.model.PolarDeviceInfo;
import polar.com.sdk.api.model.PolarHrData;

public class PolarH10Frag extends Fragment {
    private String DEVICE_ID;
    public SharedPreferences sharedPreferences;
    private String sharedPrefsKey = "polar_h10_device_id";
    private String TAG = "Polar_H10Frag";
    public PolarBleApi api;
    public Context classContext;
    public TextView textViewBattery;
    public TextView connectStatus;
    public Chronometer showStartTime;
    public ToggleButton toggle;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.polar_h10_frag, container, false);
        sharedPreferences = this.getActivity().getPreferences(Context.MODE_PRIVATE);

        // Enter device ID text field
        EditText enterIdText = (EditText) view.findViewById(R.id.editTextSetID_frag1);
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
                editor.apply();
            }
        });

        // Connection Status: start and stop toggle button
        toggle = (ToggleButton) view.findViewById(R.id.start_stop_connection_frag1);
        toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    // The toggle is enabled: Start Connection
                    onClickStartConnection(view);
                } else {
                    // The toggle is disabled: Stop Connection
                    onClickStopConnection(view);
                }
            }
        });

        //Show button that links to plots
        Button button = view.findViewById(R.id.plot_button_frag1);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Code here executes on main thread after user presses button
                checkBT();
                DEVICE_ID = sharedPreferences.getString(sharedPrefsKey,"");
                Log.d(TAG,DEVICE_ID);
                if(DEVICE_ID.equals("")){
                    showDialog(view);
                } else {
                    Toast.makeText(view.getContext(),getString(R.string.connecting) + " " + DEVICE_ID,Toast.LENGTH_SHORT).show();
                    Intent intent = new Intent(view.getContext(), PlotsActivity.class);
                    intent.putExtra("id", DEVICE_ID);
                    intent.putExtra("from_frag", TAG);
                    startActivity(intent);
                }
            }
        });

        return view;
    }

    public void checkBT(){
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBluetoothAdapter != null && !mBluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, 2);
        }

        //requestPermissions() method needs to be called when the build SDK version is 23 or above
        if(Build.VERSION.SDK_INT >= 23){
            this.requestPermissions(new String[]{Manifest.permission.ACCESS_COARSE_LOCATION,Manifest.permission.ACCESS_FINE_LOCATION},1);
        }
    }

    public void showDialog(View view){
        AlertDialog.Builder dialog = new AlertDialog.Builder(this.getContext(), R.style.PolarTheme);
        dialog.setTitle("Enter your Polar device's ID");

        View viewInflated = LayoutInflater.from(this.getActivity().getApplicationContext()).inflate(R.layout.device_id_dialog_layout,(ViewGroup) view.getRootView() , false);

        final EditText input = viewInflated.findViewById(R.id.input);
        input.setInputType(InputType.TYPE_CLASS_TEXT);
        dialog.setView(viewInflated);

        dialog.setPositiveButton("OK", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                DEVICE_ID = input.getText().toString();
                SharedPreferences.Editor editor = sharedPreferences.edit();
                editor.putString(sharedPrefsKey, DEVICE_ID);
                editor.apply();
            }
        });
        dialog.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.cancel();
            }
        });
        dialog.show();
    }

    public void onClickStartConnection(View view) {
        checkBT();
        DEVICE_ID = sharedPreferences.getString(sharedPrefsKey,"");
        Log.d(TAG,DEVICE_ID);
        if(DEVICE_ID.equals("")){
            showDialog(view);
            toggle.setChecked(false);
        } else {
            // Show that the app is trying to connect with the given device ID
            Toast.makeText(view.getContext(),getString(R.string.connecting) + " " + DEVICE_ID,Toast.LENGTH_SHORT).show();

            // Set up properties
            classContext = this.getActivity().getApplicationContext();
            textViewBattery = (TextView) view.findViewById(R.id.battery_frag1);
            connectStatus = (TextView) view.findViewById(R.id.status_frag1);

            showStartTime = (Chronometer) view.findViewById(R.id.timer_frag1);
            showStartTime.setBase(SystemClock.elapsedRealtime());
            showStartTime.setFormat("00:%s");
            showStartTime.setOnChronometerTickListener(new Chronometer.OnChronometerTickListener() {
                public void onChronometerTick(Chronometer c) {
                    long elapsedMillis = SystemClock.elapsedRealtime() -c.getBase();
                    if(elapsedMillis > 3600000L){
                        c.setFormat("0%s");
                    }else{
                        c.setFormat("00:%s");
                    }
                }
            });

            // Override some methods in api
            api = PolarBleApiDefaultImpl.defaultImplementation(this.getActivity().getApplicationContext(),
                    PolarBleApi.FEATURE_POLAR_SENSOR_STREAMING |
                            PolarBleApi.FEATURE_BATTERY_INFO |
                            PolarBleApi.FEATURE_DEVICE_INFO |
                            PolarBleApi.FEATURE_HR);
            api.setApiCallback(new PolarBleApiCallback() {
                @Override
                public void blePowerStateChanged(boolean b) {
                    Log.d(TAG, "BluetoothStateChanged " + b);
                }

                @Override
                public void deviceConnected(PolarDeviceInfo s) {
                    Log.d(TAG, "Device connected " + s.deviceId);
                    //Toast.makeText(classContext, R.string.connected,
                    //        Toast.LENGTH_SHORT).show();
                    showStartTime.start();
                    connectStatus.setText("");
                    connectStatus.append("Connected\n");
                }

                @Override
                public void deviceConnecting(PolarDeviceInfo polarDeviceInfo) {

                }

                @Override
                public void deviceDisconnected(PolarDeviceInfo s) {
                    Log.d(TAG, "Device disconnected " + s);
                    //Toast.makeText(classContext, R.string.disconnected,
                    //        Toast.LENGTH_SHORT).show();
                    showStartTime.stop();
                    showStartTime.setText("");
                    connectStatus.append("Disconnected\n");
                }

                @Override
                public void ecgFeatureReady(String s) {
                    Log.d(TAG, "ECG Feature ready " + s);
                    //streamECG();
                }

                @Override
                public void accelerometerFeatureReady(String s) {
                    Log.d(TAG, "ACC Feature ready " + s);
                }

                @Override
                public void ppgFeatureReady(String s) {
                    Log.d(TAG, "PPG Feature ready " + s);
                }

                @Override
                public void ppiFeatureReady(String s) {
                    Log.d(TAG, "PPI Feature ready " + s);
                }

                @Override
                public void biozFeatureReady(String s) {

                }

                @Override
                public void hrFeatureReady(String s) {
                    Log.d(TAG, "HR Feature ready " + s);
                }

                @Override
                public void disInformationReceived(String s, UUID u, String s1) {
                    if( u.equals(UUID.fromString("00002a28-0000-1000-8000-00805f9b34fb"))) {
                        String msg = "Firmware: " + s1.trim();
                        Log.d(TAG, "Firmware: " + s + " " + s1.trim());
                        //textViewFW.append(msg + "\n");
                    }
                }

                @Override
                public void batteryLevelReceived(String s, int i) {
                    String msg = ""+i+"%";
                    Log.d(TAG, "Battery level " + s + " " + i);
                    //Toast.makeText(classContext, msg, Toast.LENGTH_LONG).show();
                    textViewBattery.append(msg + "\n");
                }

                @Override
                public void hrNotificationReceived(String s,
                                                   PolarHrData polarHrData) {
                    Log.d(TAG, "HR " + polarHrData.hr);
                    //textViewHR.setText(String.valueOf(polarHrData.hr));
                }

                @Override
                public void polarFtpFeatureReady(String s) {
                    Log.d(TAG, "Polar FTP ready " + s);
                }
            });

            // Connect to the device
            try {
                api.connectToDevice(DEVICE_ID);
                Log.d(TAG, "finish");
            } catch (PolarInvalidArgument a){
                a.printStackTrace();
            }
        }
    }

    public void onClickStopConnection(View view) {
        try {
            api.disconnectFromDevice(DEVICE_ID);
            textViewBattery.setText("");
            connectStatus.setText("");
            Log.d(TAG, "finish");
        } catch (PolarInvalidArgument a){
            a.printStackTrace();
        }
    }


}
