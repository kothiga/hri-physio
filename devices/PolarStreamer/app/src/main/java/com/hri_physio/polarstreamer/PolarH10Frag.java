package com.hri_physio.polarstreamer;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.Uri;
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
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.core.content.FileProvider;
import androidx.fragment.app.Fragment;

import com.androidplot.xy.BoundaryMode;
import com.androidplot.xy.StepMode;
import com.androidplot.xy.XYGraphWidget;
import com.androidplot.xy.XYPlot;
import com.google.android.material.snackbar.Snackbar;

import org.reactivestreams.Publisher;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;
import java.util.UUID;

import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers;
import io.reactivex.rxjava3.disposables.Disposable;
import io.reactivex.rxjava3.functions.Function;
import io.reactivex.rxjava3.schedulers.Schedulers;
import polar.com.sdk.api.PolarBleApi;
import polar.com.sdk.api.PolarBleApiCallback;
import polar.com.sdk.api.PolarBleApiDefaultImpl;
import polar.com.sdk.api.errors.PolarInvalidArgument;
import polar.com.sdk.api.model.PolarAccelerometerData;
import polar.com.sdk.api.model.PolarDeviceInfo;
import polar.com.sdk.api.model.PolarEcgData;
import polar.com.sdk.api.model.PolarHrData;
import polar.com.sdk.api.model.PolarSensorSetting;

public class PolarH10Frag extends Fragment {
    private String DEVICE_ID;
    public SharedPreferences sharedPreferences;
    private String sharedPrefsKey = "polar_h10_device_id";
    private String TAG = "Polar_H10Frag";
    public Context classContext;
    public Activity classActivity;
    public PolarBleApi api;

    // displays: data views+chronometer
    public Boolean apiConnected = Boolean.FALSE;
    public TextView textViewBattery;
    public TextView connectStatus;
    public TextView heartRate;
    public TextView rrInterval;
    public TextView accelerometerData;
    public TextView ecgData;
    public Chronometer showStartTime;

    // set acc sensor settings
    public PolarSensorSetting sensorSetting;
    public RadioGroup radioGroupSamplingRate;
    public RadioButton Hz25;
    public RadioButton Hz50;
    public RadioButton Hz100;
    public RadioButton Hz200;
    public RadioGroup radioGroupRange;
    public RadioButton G2;
    public RadioButton G4;
    public RadioButton G8;
    public Button editSetting;

    // connect, stream and recording buttons
    public ToggleButton toggleConnection;
    public ToggleButton toggleStreamACC;
    public ToggleButton toggleStreamECG;
    public ToggleButton start_stop_recording;
    public Boolean recording = false;

    // button export acc/ecg/hr files
    public Button exportData;
    // generate acc/ecg/hr csv data header
    StringBuilder accCSV = new StringBuilder();
    StringBuilder ecgCSV = new StringBuilder();
    StringBuilder hrCSV = new StringBuilder();
    // record device start time in nanoseconds
    public long start_time;

    // LSL streaming
    public LSLStream streamHr;
    public LSLStream streamRr;
    public LSLStream streamEcg;
    public LSLStream streamAcc;
    // plots
    public RadioGroup radioGroupPlots;
    public RadioButton radioButtonHR;
    public RadioButton radioButtonECG;
    public RadioButton radioButtonACC;
    private XYPlot plotHR, plotECG, plotACC;
    private TimePlotterHR timePlotterHR;
    private PlotterACC plotterACC;
    private PlotterECG plotterECG;
    public PlotterListener plotterListener = new PlotterListener() {
        @Override
        public void update() {
            plotHR.redraw();
            plotECG.redraw();
            plotACC.redraw();
        }
    };
    public Disposable ecgDisposable;
    public Disposable accDisposable;


    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.polar_h10_frag, container, false);
        sharedPreferences = this.getActivity().getPreferences(Context.MODE_PRIVATE);

        // Set up properties
        classContext = this.getActivity().getApplicationContext();
        classActivity = this.getActivity();

        //Default to hide all plots
        plotHR = view.findViewById(R.id.plotHR);
        plotHR.setVisibility(View.GONE);
        plotECG = view.findViewById(R.id.plotECG);
        plotECG.setVisibility(View.GONE);
        plotACC = view.findViewById(R.id.plotACC);
        plotACC.setVisibility(View.GONE);

        //Set column header to to-be-exported CSV
        hrCSV.append("System Time,Internal Time,hr (bpm),rr interval (ms)");
        ecgCSV.append("System Time,Internal Time,ecg (micro volt)");
        accCSV.append("System Time,Internal Time,x (mg),y (mg),z (mg)");

        // Enter device ID text field
        EditText enterIdText = (EditText) view.findViewById(R.id.editTextSetID_frag1);
        enterIdText.setInputType(InputType.TYPE_CLASS_TEXT);
        enterIdText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
                if (sharedPreferences.getString(DEVICE_ID, null) != null) {
                    enterIdText.setHint(DEVICE_ID);
                }
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
            }

            @Override
            public void afterTextChanged(Editable s) {
                String input = enterIdText.getText().toString();
                // remove trailing white space
                input = input.replaceAll("\\s","");
                // convert lower case to upper case
                input = input.toUpperCase();
                DEVICE_ID = input;
                SharedPreferences.Editor editor = sharedPreferences.edit();
                editor.putString(sharedPrefsKey, DEVICE_ID);
                editor.apply();
            }
        });

        // Connection Status: start and stop toggle button
        toggleConnection = (ToggleButton) view.findViewById(R.id.start_stop_connection_frag1);
        toggleConnection.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
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

        // Display info from api:
        textViewBattery = (TextView) view.findViewById(R.id.battery_frag1);
        connectStatus = (TextView) view.findViewById(R.id.status_frag1);
        heartRate = (TextView) view.findViewById(R.id.hr_frag1);
        rrInterval = (TextView) view.findViewById(R.id.rr_frag1);
        ecgData = (TextView) view.findViewById(R.id.ecg_frag1);
        accelerometerData = (TextView) view.findViewById(R.id.acc_frag1);

        // Show Timer
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

        //Edit setting button for H10: set sampling frequency and range
        editSetting = (Button) view.findViewById(R.id.setting_button);
        editSetting.setOnClickListener(new Button.OnClickListener(){
            @Override
            public void onClick(View v) {
                showSettingDialog(v);
            }
        });

        //button linking to ECG streaming
        toggleStreamECG = (ToggleButton) view.findViewById(R.id.stream_ECG_button_frag1);
        toggleStreamECG.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to stream data. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    toggleStreamECG.setChecked(false);
                }
                else {
                    if (isChecked) {
                        // create ECG stream
                        streamEcg = new LSLStream();
                        // streaming LSL
                        // declare info strings to store stream data info for LSL
                        // info input in format of: { [0] "device name in format: PolarDeviceName/DeviceID/service", [1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id(=device name)"}
                        String deviceNameEcg = "PolarH10/"+DEVICE_ID+"/ECG";
                        String[] ecgInfo = new String[]{deviceNameEcg, "ECG", "1", "130", deviceNameEcg};
                        try {
                            streamEcg.StreamOutlet(ecgInfo);
                        } catch (IOException e) {
                            e.printStackTrace();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        ecgData.setText("loading data...");
                        //stream data, add to plot and write to file.
                        if(ecgDisposable == null) {
                            ecgDisposable = api.requestEcgSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarEcgData>>) settings -> {
                                PolarSensorSetting sensorSetting = settings.maxSettings();
                                return api.startEcgStreaming(DEVICE_ID, sensorSetting);
                            }).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    polarEcgData -> {
                                        ecgData.setText(String.valueOf(polarEcgData.samples.get(0) / 1000.0));
                                        plotterECG.sendList(polarEcgData.samples);
                                        for (Integer data : polarEcgData.samples) {
                                            if (recording) {
                                                SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                                                sdf.setTimeZone(TimeZone.getDefault());
                                                String currentDateAndTime = sdf.format(new Date());
                                                ecgCSV.append("\n"+currentDateAndTime+","+getElapsedNanoTime() + ","+data);
                                            }
                                        }
                                        streamEcg.runList(polarEcgData.samples);
                                    },
                                    throwable -> Log.e(TAG,""+throwable.getLocalizedMessage()),
                                    () -> Log.d(TAG,"complete")
                            );
                        } else {
                            // NOTE dispose will stop streaming if it is "running"
                            ecgDisposable.dispose();
                            ecgDisposable = null;
                            isChecked = false;
                        }
                        //show plot
                        showPlotECG(view);
                        plotECG.clear();
                        plotECG.setVisibility(View.GONE);
                    } else {
                        // NOTE dispose will stop streaming if it is "running"
                        if(ecgDisposable != null){
                            ecgDisposable.dispose();
                            ecgDisposable = null;
                        }
                        ecgData.setText("");
                        if(streamEcg.outlet != null) streamEcg.close();
                    }
                }
            }
        });

        //button linking to ACC streaming
        toggleStreamACC = (ToggleButton) view.findViewById(R.id.stream_ACC_button_frag1);
        toggleStreamACC.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to stream data. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    toggleStreamACC.setChecked(false);
                }
                else {
                    if (isChecked) {
                        //create ACC stream
                        streamAcc = new LSLStream();
                        // streaming LSL
                        // declare info strings to store stream data info for LSL
                        // info input in format of: { [0] "device name in format: PolarDeviceName/DeviceID/service", [1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id(=device name)"}
                        String deviceNameAcc = "PolarH10/"+DEVICE_ID+"/ACC";
                        String[] accInfo = new String[]{deviceNameAcc, "ACC", "3", "200", deviceNameAcc};
                        if(sensorSetting!= null){
                            accInfo[3] = String.valueOf(sensorSetting.settings.get(PolarSensorSetting.SettingType.SAMPLE_RATE));
                        }
                        try {
                            streamAcc.StreamOutlet(accInfo);
                        } catch (IOException e) {
                            e.printStackTrace();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        accelerometerData.setText("loading data...");
                        if(accDisposable == null) {
                            accDisposable = api.requestAccSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarAccelerometerData>>) settings -> {
                                PolarSensorSetting currentSetting;
                                if (sensorSetting!=null){
                                    currentSetting = sensorSetting;
                                }
                                else{
                                    currentSetting = settings.maxSettings();
                                }
                                return api.startAccStreaming(DEVICE_ID, currentSetting);
                            }).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    polarAccData -> {
                                        accelerometerData.setText("x: " + polarAccData.samples.get(0).x + " y: " + polarAccData.samples.get(0).y + " z: "+ polarAccData.samples.get(0).z);
                                        plotterACC.addValues(polarAccData.samples.get(0));
                                        if(recording){
                                            for(PolarAccelerometerData.PolarAccelerometerSample sample: polarAccData.samples){
                                                SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                                                sdf.setTimeZone(TimeZone.getDefault());
                                                String currentDateAndTime = sdf.format(new Date());
                                                accCSV.append("\n"+currentDateAndTime+","+ getElapsedNanoTime() + "," + sample.x+","+ sample.y+","+sample.z);
                                            }
                                        }
                                        streamAcc.runAcc(polarAccData.samples);
                                    },
                                    throwable -> Log.e(TAG,""+throwable.getLocalizedMessage()),
                                    () -> Log.d(TAG,"complete")
                            );
                        } else {
                            // NOTE dispose will stop streaming if it is "running"
                            accDisposable.dispose();
                            accDisposable = null;
                            isChecked = false;
                        }
                        //show plot
                        showPlotACC(view);
                        plotACC.clear();
                        plotACC.setVisibility(View.GONE);
                    } else {
                        if (accDisposable != null) {
                            accDisposable.dispose();
                            accDisposable = null;
                        }
                        accelerometerData.setText("");
                        if(streamAcc.outlet != null) streamAcc.close();
                    }
                }
            }
        });

        // Start and stop recording data to CSV
        start_stop_recording = (ToggleButton) view.findViewById(R.id.start_stop_recording_frag1);
        start_stop_recording.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    recording = true;
                } else {
                    recording = false;
                }
            }
        });

        // export data
        exportData = (Button) view.findViewById(R.id.export_data);
        exportData.setOnClickListener(v -> {
            showExportDialogue(view);
        });

        radioGroupPlots = (RadioGroup) view.findViewById(R.id.radioGroupPlots);
        radioButtonHR = (RadioButton) view.findViewById(R.id.radioButtonHR);
        radioButtonECG = (RadioButton) view.findViewById(R.id.radioButtonECG);
        radioButtonACC = (RadioButton) view.findViewById(R.id.radioButtonACC);

        radioGroupPlots.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                // find which radio button is selected
                if(checkedId == R.id.radioButtonHR) {
                    plotECG.clear();
                    plotECG.setVisibility(View.GONE);
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
                    showPlotHR(view);
                } else if(checkedId == R.id.radioButtonECG) {
                    plotHR.clear();
                    plotHR.setVisibility(View.GONE);
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
                    showPlotECG(view);
                }
                else if(checkedId == R.id.radioButtonACC){
                    plotHR.clear();
                    plotHR.setVisibility(View.GONE);
                    plotECG.clear();
                    plotECG.setVisibility(View.GONE);
                    showPlotACC(view);
                }
                else {
                    plotHR.clear();
                    plotHR.setVisibility(View.GONE);
                    plotECG.clear();
                    plotECG.setVisibility(View.GONE);
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
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
                Toast.makeText(classContext, R.string.searchBattery,
                        Toast.LENGTH_SHORT).show();
                showStartTime.start();
                connectStatus.setText("");
                apiConnected = Boolean.TRUE;
                connectStatus.append("Connected\n");
                heartRate.setText("loading data...");
                rrInterval.setText("loading data...");
                start_time = System.nanoTime();
            }

            @Override
            public void deviceConnecting(PolarDeviceInfo s) {
            }

            @Override
            public void deviceDisconnected(PolarDeviceInfo s) {
                Toast.makeText(classContext, R.string.disconnected,
                        Toast.LENGTH_SHORT).show();
                apiConnected = Boolean.FALSE;
                if(streamHr.outlet != null) streamHr.close();
                if(streamRr.outlet != null) streamRr.close();
            }

            @Override
            public void ecgFeatureReady(String s) {
                Log.d(TAG, "ECG Feature ready " + s);
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
            public void biozFeatureReady(String s) { Log.d(TAG, "Bioz Feature ready " + s);  }

            @Override
            public void hrFeatureReady(String s) {
                Log.d(TAG, "HR Feature ready " + s);
                //Create HR stream if HR is ready
                streamHr = new LSLStream();
                // info input in format of: { [0] "device name in format: PolarDeviceName/DeviceID/service", [1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id(=device name)"}
                String deviceNameHr = "PolarH10/"+DEVICE_ID+"/HR";
                String[] hrInfo = new String[]{deviceNameHr, "HR", "1", "1", deviceNameHr};
                try {
                    streamHr.StreamOutlet(hrInfo);
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                //create RR stream
                streamRr = new LSLStream();
                String deviceNameRr = "PolarH10/"+DEVICE_ID+"/RR";
                String[] rrInfo = new String[]{deviceNameRr, "RR", "1", "2", deviceNameRr};
                try {
                    streamRr.StreamOutlet(rrInfo);
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                //show plot for HR
                showPlotHR(view);
                plotHR.clear();
                plotHR.setVisibility(View.GONE);
            }

            @Override
            public void disInformationReceived(String s, UUID u, String s1) {
                if( u.equals(UUID.fromString("00002a28-0000-1000-8000-00805f9b34fb"))) {
                    String msg = "Firmware: " + s1.trim();
                    Log.d(TAG, "Firmware: " + s + " " + s1.trim());
                }
            }

            @Override
            public void batteryLevelReceived(String s, int i) {
                String msg = ""+i+"%";
                Log.d(TAG, "Battery level " + s + " " + i);
                textViewBattery.append(msg + "\n");
            }

            @Override
            public void hrNotificationReceived(String s,
                                               PolarHrData polarHrData) {
                Log.d(TAG, "HR " + polarHrData.hr);
                heartRate.setText(String.valueOf(polarHrData.hr));
                timePlotterHR.addValues(polarHrData);

                // When rr interval is available
                if(!polarHrData.rrsMs.isEmpty()){
                    // Update set text for RR
                    for (Integer rr : polarHrData.rrsMs)
                        rrInterval.setText(String.valueOf(rr));
                    //stream to LSL: overloaded runHr method
                    try {
                        streamHr.runHr(polarHrData.hr);
                        streamRr.runList(polarHrData.rrsMs);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                else{
                    //Only stream HR
                    try {
                        streamHr.runHr(polarHrData.hr);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                // edit hrCSV
                if (recording) {
                    SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                    sdf.setTimeZone(TimeZone.getDefault());
                    String currentDateAndTime = sdf.format(new Date());
                    hrCSV.append("\n"+currentDateAndTime+","+getElapsedNanoTime() + ","+polarHrData.hr+",");
                    if(polarHrData.rrsMs.size() != 0){
                        // only add the max RR interval value for one second.
                        hrCSV.append(Collections.max(polarHrData.rrsMs));
//                        for(int i = 0; i < polarHrData.rrsMs.size(); i++){
//                            hrCSV.append(polarHrData.rrsMs.get(i)+" ");
//                        }
                    }
                }
            }

            @Override
            public void polarFtpFeatureReady(String s) {
                Log.d(TAG, "Polar FTP ready " + s);
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

    // Sensor ID dialog: if device ID is not entered, show dialog to ask for ID input.
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

    // Sensor setting dialog: allows user to set sampling rate and range for ACC streaming.
    public void showSettingDialog(View view){
        AlertDialog.Builder dialog = new AlertDialog.Builder(this.getContext(), R.style.PolarTheme);
        PolarSensorSetting.Builder builder = PolarSensorSetting.Builder.newBuilder();
        dialog.setTitle("Sensor Setting (ACC)");

        View viewInflated = LayoutInflater.from(this.getActivity().getApplicationContext()).inflate(R.layout.h10_setting_dialog_layout,(ViewGroup) view.getRootView() , false);

        dialog.setView(viewInflated);

        radioGroupSamplingRate = (RadioGroup) viewInflated.findViewById(R.id.radioGroupSamplingRate);
        Hz25 = (RadioButton) viewInflated.findViewById(R.id.Hz25);
        Hz50 = (RadioButton) viewInflated.findViewById(R.id.Hz50);
        Hz100 = (RadioButton) viewInflated.findViewById(R.id.Hz100);
        Hz200 = (RadioButton) viewInflated.findViewById(R.id.Hz200);

        radioGroupSamplingRate.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                // find which radio button is selected
                if(checkedId == R.id.Hz25) {
                    Toast.makeText(viewInflated.getContext(), "Sampling Rate = 25 Hz",
                            Toast.LENGTH_SHORT).show();
                } else if(checkedId == R.id.Hz50) {
                    Toast.makeText(viewInflated.getContext(), "Sampling Rate = 50 Hz",
                            Toast.LENGTH_SHORT).show();
                } else if(checkedId == R.id.Hz100) {
                    Toast.makeText(viewInflated.getContext(), "Sampling Rate = 100 Hz",
                            Toast.LENGTH_SHORT).show();
                }
                else {
                    Toast.makeText(viewInflated.getContext(), "Sampling Rate = 200 Hz",
                            Toast.LENGTH_SHORT).show();
                }
            }
        });

        radioGroupRange = (RadioGroup) viewInflated.findViewById(R.id.radioGroupRange);
        G2 = (RadioButton) viewInflated.findViewById(R.id.G2);
        G4 = (RadioButton) viewInflated.findViewById(R.id.G4);
        G8 = (RadioButton) viewInflated.findViewById(R.id.G8);

        radioGroupRange.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                // find which radio button is selected
                if(checkedId == R.id.G2) {
                    Toast.makeText(viewInflated.getContext(), "Range = 2 g",
                            Toast.LENGTH_SHORT).show();
                } else if(checkedId == R.id.G4) {
                    Toast.makeText(viewInflated.getContext(), "Range = 4 g",
                            Toast.LENGTH_SHORT).show();
                } else if(checkedId == R.id.G8) {
                    Toast.makeText(viewInflated.getContext(), "Range = 8 g",
                            Toast.LENGTH_SHORT).show();
                }
            }
        });

        dialog.setPositiveButton("OK", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                int selectedSamplingRateId = radioGroupSamplingRate.getCheckedRadioButtonId();
                int selectedRangeId = radioGroupRange.getCheckedRadioButtonId();
                if (selectedSamplingRateId == Hz25.getId()){
                    builder.setSampleRate(25);
                }
                else if (selectedSamplingRateId == Hz50.getId()){
                    builder.setSampleRate(50);
                }
                else if (selectedSamplingRateId == Hz100.getId()){
                    builder.setSampleRate(100);
                }
                else {
                    builder.setSampleRate(200);
                }

                if (selectedRangeId == G2.getId()){
                    builder.setRange(2);
                }
                else if (selectedRangeId == G4.getId()){
                    builder.setRange(4);
                }
                else {
                    builder.setRange(8);
                }
                //default not customizable.
                builder.setResolution(16);
                sensorSetting = builder.build();
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
            toggleConnection.setChecked(false);
        } else {
            // Show that the app is trying to connect with the given device ID
            Toast.makeText(view.getContext(),getString(R.string.connecting) + " " + DEVICE_ID,Toast.LENGTH_SHORT).show();

            // reset chronometer time
            showStartTime.setBase(SystemClock.elapsedRealtime());

            // Connect to the device
            try {
                api.connectToDevice(DEVICE_ID);
                apiConnected = Boolean.TRUE;
                Log.d(TAG, "finish");
            } catch (PolarInvalidArgument a){
                a.printStackTrace();
            }
        }
    }

    public void showPlotHR(View view){
        plotHR.setVisibility(View.VISIBLE);
        // Plot HR/RR graph
        timePlotterHR = new TimePlotterHR(classContext, "HR/RR");
        timePlotterHR.setListener(plotterListener);
        plotHR.addSeries(timePlotterHR.getHrSeries(), timePlotterHR.getHrFormatter());
        plotHR.addSeries(timePlotterHR.getRrSeries(), timePlotterHR.getRrFormatter());
        plotHR.setRangeBoundaries(50, 100,
                BoundaryMode.AUTO);
        plotHR.setDomainBoundaries(0, 360000,
                BoundaryMode.AUTO);
        plotHR.setRangeStep(StepMode.SUBDIVIDE, 5);
        plotHR.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
        // Make left labels be an integer (no decimal places)
        plotHR.getGraph().getLineLabelStyle(XYGraphWidget.Edge.LEFT).
                setFormat(new DecimalFormat("#"));
        plotHR.getLegend().setVisible(true);
    }

    public void showPlotECG(View view){
        plotECG.setVisibility(View.VISIBLE);
        //Plot ECG graph
        plotterECG = new PlotterECG(classContext, "ECG");
        plotterECG.setListener(plotterListener);

        plotECG.addSeries(plotterECG.getSeries(), plotterECG.getFormatter());
        plotECG.setRangeBoundaries(-4, 4, BoundaryMode.FIXED);
        plotECG.setDomainBoundaries(0, 500, BoundaryMode.FIXED);

        plotACC.setRangeStep(StepMode.SUBDIVIDE, 10);
        plotACC.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
    }

    public void showPlotACC(View view){
        plotACC.setVisibility(View.VISIBLE);
//        //Plot ACC graph
        plotterACC = new PlotterACC(classContext, "ACC");
        plotterACC.setListener(plotterListener);
        plotACC.addSeries(plotterACC.getAccXSeries(), plotterACC.getAccXFormatter());
        plotACC.addSeries(plotterACC.getAccYSeries(), plotterACC.getAccYFormatter());
        plotACC.addSeries(plotterACC.getAccZSeries(), plotterACC.getAccZFormatter());

        plotACC.setRangeBoundaries(-1000, 1000,
                BoundaryMode.AUTO);
        plotACC.setDomainBoundaries(0, 300,
                BoundaryMode.FIXED);
        // Left labels
        plotACC.setRangeStep(StepMode.SUBDIVIDE, 10);
        plotACC.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
        // Make left labels be an integer (no decimal places)
        plotACC.getGraph().getLineLabelStyle(XYGraphWidget.Edge.LEFT).
                setFormat(new DecimalFormat("#"));
        plotACC.getLegend().setVisible(true);
    }

    public void onClickStopConnection(View view) {
        try {
            api.disconnectFromDevice(DEVICE_ID);

            textViewBattery.setText("");
            connectStatus.setText("");
            connectStatus.append("Disconnected\n");
            showStartTime.stop();
            showStartTime.setText("");
            heartRate.setText("");
            rrInterval.setText("");
            accelerometerData.setText("");
            ecgData.setText("");
            accDisposable = null;
            ecgDisposable = null;

            toggleStreamECG.setChecked(false);
            toggleStreamACC.setChecked(false);
            radioGroupPlots.clearCheck();
            plotHR.setVisibility(View.GONE);
            plotECG.setVisibility(View.GONE);
            plotACC.setVisibility(View.GONE);

            recording = false;
            Log.d(TAG, "finish");
        } catch (PolarInvalidArgument a){
            a.printStackTrace();
        }

    }

    public void showExportDialogue(View view) {
        try{

            // saving hr file into device
            FileOutputStream outHr = classActivity.openFileOutput("hr data.csv", Context.MODE_PRIVATE);
            outHr.write((hrCSV.toString()).getBytes());
            outHr.close();

            // saving acc file into device
            FileOutputStream outAcc = classActivity.openFileOutput("acc data.csv", Context.MODE_PRIVATE);
            outAcc.write((accCSV.toString()).getBytes());
            outAcc.close();

            // saving ecg file into device
            FileOutputStream outEcg = classActivity.openFileOutput("ecg data.csv", Context.MODE_PRIVATE);
            outEcg.write((ecgCSV.toString()).getBytes());
            outEcg.close();

            // get hr file location
            File fileLocationHr = new File(classContext.getFilesDir(), "hr data.csv");
            Uri pathHr = FileProvider.getUriForFile(classContext, "com.hri_physio.polarstreamer", fileLocationHr);

            // get acc file location
            File fileLocationAcc = new File(classActivity.getFilesDir(), "acc data.csv");
            Uri pathAcc = FileProvider.getUriForFile(classContext, "com.hri_physio.polarstreamer", fileLocationAcc);

            // get ecg file location
            File fileLocationEcg = new File(classActivity.getFilesDir(), "ecg data.csv");
            Uri pathEcg = FileProvider.getUriForFile(classContext, "com.hri_physio.polarstreamer", fileLocationEcg);

            // start exporting
            Intent fileIntent = new Intent(Intent.ACTION_SEND_MULTIPLE);
            fileIntent.setType("text/csv");
            fileIntent.putExtra(Intent.EXTRA_SUBJECT, "Data");
            fileIntent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
            fileIntent.addFlags(Intent.FLAG_GRANT_WRITE_URI_PERMISSION);

            // send multiple attachments
            ArrayList<Uri> uris = new ArrayList<Uri>();
            uris.add(pathHr);
            uris.add(pathAcc);
            uris.add(pathEcg);
            fileIntent.putExtra(Intent.EXTRA_STREAM, uris);
            startActivity(Intent.createChooser(fileIntent, "Send Polar H10 device data"));

        }
        catch(Exception e){
            e.printStackTrace();
        }
    }
    public String getElapsedNanoTime () {
        long elapsed_time = System.nanoTime() - start_time;
        StringBuilder sb = new StringBuilder();
        long seconds = elapsed_time / 1000000000;
        long days = seconds / (3600 * 24);
        sb.append(days+"d");
        seconds -= (days * 3600 * 24);
        long hours = seconds / 3600;
        sb.append(hours+"h");
        seconds -= (hours * 3600);
        long minutes = seconds / 60;
        sb.append(minutes+"m");
        seconds -= (minutes * 60);
        sb.append(seconds+"s");
        long nanos = elapsed_time % 1000000000;
        sb.append(nanos+"ns");
        return sb.toString();
    }

}
