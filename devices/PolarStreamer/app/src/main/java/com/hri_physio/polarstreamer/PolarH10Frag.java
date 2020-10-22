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
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.fragment.app.Fragment;
import com.androidplot.xy.BoundaryMode;
import com.androidplot.xy.StepMode;
import com.androidplot.xy.XYGraphWidget;
import com.androidplot.xy.XYPlot;
import com.google.android.material.snackbar.Snackbar;
import org.reactivestreams.Publisher;
import java.text.DecimalFormat;
import java.util.UUID;
import io.reactivex.rxjava3.disposables.Disposable;
import io.reactivex.rxjava3.functions.Action;
import io.reactivex.rxjava3.functions.Consumer;
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
    public PolarBleApi api;
    public Context classContext;
    public TextView textViewBattery;
    public TextView connectStatus;

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
    public Chronometer showStartTime;
    public ToggleButton toggle;
    public Button editSetting;
    public ToggleButton togglePlotHR;
    public ToggleButton togglePlotACC;
    public ToggleButton togglePlotECG;
    public Boolean apiConnected = Boolean.FALSE;

    private XYPlot plotHR, plotECG, plotACC;
    private TimePlotterHR timePlotterHR;
    private TimePlotterACC timeplotterACC;
    private TimePlotterECG timeplotterECG;
    public PlotterListener plotterListener = new PlotterListener() {
        @Override
        public void update() {
            plotHR.redraw();
            plotECG.redraw();
            plotACC.redraw();
        }
    };
    private Disposable ecgDisposable = null;
    private Disposable accDisposable = null;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.polar_h10_frag, container, false);
        sharedPreferences = this.getActivity().getPreferences(Context.MODE_PRIVATE);

        //Default to hide all plots at first
        plotHR = view.findViewById(R.id.plotHR);
        plotHR.setVisibility(View.GONE);
        plotECG = view.findViewById(R.id.plotECG);
        plotECG.setVisibility(View.GONE);
        plotACC = view.findViewById(R.id.plotACC);
        plotACC.setVisibility(View.GONE);


        // Enter device ID text field
        EditText enterIdText = (EditText) view.findViewById(R.id.editTextSetID_frag1);
        enterIdText.setInputType(InputType.TYPE_CLASS_TEXT);
        enterIdText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
            }

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

        //Edit setting button for H10: set sampling frequency and range
        editSetting = (Button) view.findViewById(R.id.setting_button);
        editSetting.setOnClickListener(new Button.OnClickListener(){
            @Override
            public void onClick(View v) {
                showSettingDialog(v);
            }
        });

        //button linking to HR plot
        togglePlotHR = (ToggleButton) view.findViewById(R.id.plot_HR_button_frag1);
        togglePlotHR.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    togglePlotHR.setChecked(false);
                }
                else {
                    if (isChecked) {
                        //show plot
                        showPlotHR(view);
                    } else {
                        // hide plot
                        plotHR.clear();
                        plotHR.setVisibility(View.GONE);
                    }
                }
            }
        });

        //button linking to ECG plot
        togglePlotECG = (ToggleButton) view.findViewById(R.id.plot_ECG_button_frag1);
        togglePlotECG.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    togglePlotECG.setChecked(false);
                }
                else {
                    if (isChecked) {
                        //show plot
                        showPlotECG(view);
                    } else {
                        // hide plot
                        plotECG.clear();
                        plotECG.setVisibility(View.GONE);
                    }
             }
            }
        });

        //button linking to ACC plot
        togglePlotACC = (ToggleButton) view.findViewById(R.id.plot_ACC_button_frag1);
        togglePlotACC.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    togglePlotACC.setChecked(false);
                }
                else {
                if (isChecked) {
                    //show plot
                    showPlotACC(view);
                } else {
                    // hide plot
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
                }
             }
            }
        });
        return view;
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
        dialog.setTitle("Sensor Setting");

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
                    Toast.makeText(classContext, R.string.connected,
                            Toast.LENGTH_SHORT).show();
                    showStartTime.start();
                    connectStatus.setText("");
                    connectStatus.append("Connected\n");
                    apiConnected = Boolean.TRUE;
                }

                @Override
                public void deviceConnecting(PolarDeviceInfo s) {
                    Log.d(TAG, "Device connecting " + s.deviceId);
                }

                @Override
                public void deviceDisconnected(PolarDeviceInfo s) {
                    Log.d(TAG, "Device disconnected " + s);
                    Toast.makeText(classContext, R.string.disconnected,
                            Toast.LENGTH_SHORT).show();
                    ecgDisposable = null;
                    accDisposable = null;
//                    ppgDisposable = null;
//                    ppiDisposable = null;
                    showStartTime.stop();
                    showStartTime.setText("");
                    connectStatus.append("Disconnected\n");
                    apiConnected = Boolean.FALSE;
                }

                @Override
                public void ecgFeatureReady(String s) {
                    Log.d(TAG, "ECG Feature ready " + s);
                    plotECG();
                }

                @Override
                public void accelerometerFeatureReady(String s) {
                    Log.d(TAG, "ACC Feature ready " + s);
                    plotACC();
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
                    textViewBattery.append(msg + "\n");
                }

                @Override
                public void hrNotificationReceived(String s,
                                                   PolarHrData polarHrData) {
                    Log.d(TAG, "HR " + polarHrData.hr);
                    timePlotterHR.addValues(polarHrData);
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
        timeplotterECG = new TimePlotterECG(classContext, "ECG");
        timeplotterECG.setListener(plotterListener);

        plotECG.addSeries(timeplotterECG.getEcgSeries(), timeplotterECG.getEcgFormatter());
        plotECG.setRangeBoundaries(-3.3, 3.3, BoundaryMode.FIXED);
        plotECG.setDomainBoundaries(0, 360000, BoundaryMode.AUTO);

        plotACC.setRangeStep(StepMode.SUBDIVIDE, 10);
        plotACC.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
    }

    public void showPlotACC(View view){
        plotACC.setVisibility(View.VISIBLE);
        //Plot ACC graph
        timeplotterACC = new TimePlotterACC(classContext, "ACC");
        timeplotterACC.setListener(plotterListener);
        plotACC.addSeries(timeplotterACC.getAccXSeries(), timeplotterACC.getAccXFormatter());
        plotACC.addSeries(timeplotterACC.getAccYSeries(), timeplotterACC.getAccYFormatter());
        plotACC.addSeries(timeplotterACC.getAccZSeries(), timeplotterACC.getAccZFormatter());
        plotACC.setRangeBoundaries(-1000, 1000,
                BoundaryMode.AUTO);
        plotACC.setDomainBoundaries(0, 360000,
                BoundaryMode.AUTO);
        // Left labels
        plotACC.setRangeStep(StepMode.SUBDIVIDE, 10);
        plotACC.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
        // Make left labels be an integer (no decimal places)
        plotACC.getGraph().getLineLabelStyle(XYGraphWidget.Edge.LEFT).
                setFormat(new DecimalFormat("#"));
        plotACC.getLegend().setVisible(true);
    }

    private void plotECG(){
        api.requestEcgSettings(DEVICE_ID).toFlowable().flatMap(new Function<PolarSensorSetting, Publisher<PolarEcgData>>() {
            @Override
            public Publisher<PolarEcgData> apply(PolarSensorSetting polarSensorSetting) throws Exception {
                return api.startEcgStreaming(DEVICE_ID, polarSensorSetting.maxSettings());
            }
        }).subscribeOn(Schedulers.newThread()).subscribe(
                new Consumer<PolarEcgData>() {
                    @Override
                    public void accept(PolarEcgData polarEcgData) throws Exception {
                        Log.d(TAG, "ecg update");
                        for (Integer data : polarEcgData.samples) {
                            timeplotterECG.addValues((float) ((float) data / 1000.0));
                        }
                    }
                },
                new Consumer<Throwable>() {
                    @Override
                    public void accept(Throwable throwable) throws Exception {
                        Log.e(TAG,
                                "" + throwable.getLocalizedMessage());
                        ecgDisposable = null;
                    }
                },
                new Action() {
                    @Override
                    public void run() throws Exception {
                        Log.d(TAG, "complete");
                    }
                }
        );
    }

    private void plotACC() {
        api.requestAccSettings(DEVICE_ID).toFlowable().flatMap(new Function<PolarSensorSetting, Publisher<PolarAccelerometerData>>() {
            @Override
            public Publisher<PolarAccelerometerData> apply(PolarSensorSetting polarSensorSetting) throws Throwable {
                if (sensorSetting != null){
                    return api.startAccStreaming(DEVICE_ID, sensorSetting.maxSettings());
                }
                return api.startAccStreaming(DEVICE_ID, polarSensorSetting.maxSettings());
            }}

            ).subscribeOn(Schedulers.newThread())
                .subscribe(
                new Consumer<PolarAccelerometerData>() {
                @Override
                public void accept(PolarAccelerometerData polarAccData) throws Exception {
                    Log.d(TAG, "acceloremeter update");
                    for (PolarAccelerometerData.PolarAccelerometerSample data : polarAccData.samples) {
                        timeplotterACC.addValues(data);
                    }
                }
            },
                new Consumer<Throwable>() {
                    @Override
                    public void accept(Throwable throwable) throws Exception {
                        Log.e(TAG,
                                "" + throwable.getLocalizedMessage());
                        accDisposable = null;
                    }
                },
                new Action() {
                    @Override
                    public void run() throws Exception {
                        Log.d(TAG, "complete");
                    }
                }
        );
    }

    public void onClickStopConnection(View view) {
        try {
            api.disconnectFromDevice(DEVICE_ID);
            textViewBattery.setText("");
            connectStatus.setText("");
            apiConnected = Boolean.FALSE;
            Log.d(TAG, "finish");
        } catch (PolarInvalidArgument a){
            a.printStackTrace();
        }
        togglePlotHR.setChecked(false);
        plotHR.setVisibility(View.GONE);
        togglePlotECG.setChecked(false);
        plotECG.setVisibility(View.GONE);
        togglePlotACC.setChecked(false);
        plotACC.setVisibility(View.GONE);
    }

}
