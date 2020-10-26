package com.hri_physio.polarstreamer;

import android.Manifest;
import android.app.Activity;
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
    public PolarBleApi api;
    public Boolean apiConnected = Boolean.FALSE;
    public TextView textViewBattery;
    public TextView connectStatus;
    public TextView heartRate;
    public TextView accelerometerData;
    public TextView ecgData;

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
    public Chronometer showStartTime;
    public ToggleButton toggle;
    public ToggleButton toggleStreamACC;
    public ToggleButton toggleStreamECG;

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
    public Activity classActivity;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.polar_h10_frag, container, false);
        sharedPreferences = this.getActivity().getPreferences(Context.MODE_PRIVATE);

        //Default to hide all plots
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

        // Set up properties
        classContext = this.getActivity().getApplicationContext();
        classActivity = this.getActivity();

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

        // Display info from api:
        textViewBattery = (TextView) view.findViewById(R.id.battery_frag1);
        connectStatus = (TextView) view.findViewById(R.id.status_frag1);
        heartRate = (TextView) view.findViewById(R.id.hr_frag1);
        accelerometerData = (TextView) view.findViewById(R.id.acc_frag1);
        ecgData = (TextView) view.findViewById(R.id.ecg_frag1);

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
                        //show plot
                        if(ecgDisposable == null) {
                            ecgDisposable = api.requestEcgSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarEcgData>>) settings -> {
                                PolarSensorSetting sensorSetting = settings.maxSettings();
                                return api.startEcgStreaming(DEVICE_ID, sensorSetting);
                            }).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    polarEcgData -> {
                                        ecgData.setText((float) ((float) polarEcgData.samples.get(0) / 1000.0) + "mV");
                                        for (Integer data : polarEcgData.samples) {
                                            plotterECG.sendSingleSample((float) ((float) data / 1000.0));
                                        }
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
                    } else {
                        // NOTE dispose will stop streaming if it is "running"
                        ecgDisposable.dispose();
                        ecgDisposable = null;
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
                    if(accDisposable == null) {
                        accDisposable = api.requestAccSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarAccelerometerData>>) settings -> {
                            if (sensorSetting!=null){
                                return api.startAccStreaming(DEVICE_ID, sensorSetting);
                            }
                            return api.startAccStreaming(DEVICE_ID, settings.maxSettings());
                        }).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                polarAccelerometerData -> {
                                    accelerometerData.setText("x: "
                                            + polarAccelerometerData.samples.get(0).x + "mG "
                                            + "y: " + polarAccelerometerData.samples.get(0).y + "mG "
                                            + "z: " + polarAccelerometerData.samples.get(0).z + "mG ");
                                    plotterACC.addValues(polarAccelerometerData.samples.get(0));
                                },
                                throwable -> Log.e(TAG,""+throwable.getLocalizedMessage()),
                                () -> Log.d(TAG,"complete")
                        );
                    } else {
                        // NOTE dispose will stop streaming if it is "running"
                        accDisposable.dispose();
                        accDisposable = null;
                    }
                } else {
                    // hide plot
                    accDisposable.dispose();
                    accDisposable = null;
                }
             }
            }
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
                    showPlotHR(view);
                } else if(checkedId == R.id.radioButtonECG) {
                    showPlotECG(view);
                }
                else if(checkedId == R.id.radioButtonACC){
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
                accelerometerData.setText("loading data...");
                ecgData.setText("loading data...");
            }

            @Override
            public void deviceConnecting(PolarDeviceInfo s) {
            }

            @Override
            public void deviceDisconnected(PolarDeviceInfo s) {
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
                timePlotterHR.addValues(polarHrData);
                heartRate.setText(String.valueOf(polarHrData.hr)+"bpm");
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
        plotACC.clear();
        plotACC.setVisibility(View.GONE);
        plotECG.clear();
        plotECG.setVisibility(View.GONE);
        plotHR.clear();
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
        plotHR.clear();
        plotHR.setVisibility(View.GONE);
        plotACC.clear();
        plotACC.setVisibility(View.GONE);
        plotECG.clear();
        plotECG.setVisibility(View.VISIBLE);
        //Plot ECG graph
        plotterECG = new PlotterECG(classContext, "ECG");
        plotterECG.setListener(plotterListener);

        plotECG.addSeries(plotterECG.getSeries(), plotterECG.getFormatter());
        plotECG.setRangeBoundaries(-4, 4, BoundaryMode.FIXED);
        plotECG.setDomainBoundaries(0, 800, BoundaryMode.FIXED);

        plotACC.setRangeStep(StepMode.SUBDIVIDE, 10);
        plotACC.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
    }

    public void showPlotACC(View view){
        plotHR.clear();
        plotHR.setVisibility(View.GONE);
        plotECG.clear();
        plotECG.setVisibility(View.GONE);
        plotACC.clear();
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
            apiConnected = Boolean.FALSE;

            textViewBattery.setText("");
            connectStatus.setText("");
            connectStatus.append("Disconnected\n");
            showStartTime.stop();
            showStartTime.setText("");
            heartRate.setText("");
            accelerometerData.setText("");
            ecgData.setText("");
            accDisposable = null;
            ecgDisposable = null;

            toggleStreamECG.setChecked(false);
            toggleStreamACC.setChecked(false);
            radioGroupPlots.clearCheck();
            Log.d(TAG, "finish");
        } catch (PolarInvalidArgument a){
            a.printStackTrace();
        }

    }

}
