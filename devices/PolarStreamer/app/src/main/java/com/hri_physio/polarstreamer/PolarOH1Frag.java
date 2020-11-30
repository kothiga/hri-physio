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
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;
import java.util.UUID;
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers;
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
import polar.com.sdk.api.model.PolarHrData;
import polar.com.sdk.api.model.PolarOhrPPGData;
import polar.com.sdk.api.model.PolarOhrPPIData;
import polar.com.sdk.api.model.PolarSensorSetting;

public class PolarOH1Frag extends Fragment {
    private String DEVICE_ID;
    public SharedPreferences sharedPreferences;
    private String sharedPrefsKey = "polar_oh1_device_id";
    private String TAG = "Polar_OH1Frag";
    public PolarBleApi api;
    public Context classContext;
    public Activity classActivity;
    public TextView textViewBattery;
    public TextView connectStatus;
    public TextView heartRate;
    public TextView accelerometerData;
    public TextView ppgData;
    public TextView ppiData;
    public Chronometer showStartTime;
    public ToggleButton toggle;
    public Disposable accDisposable = null;
    public Disposable ppgDisposable = null;
    public Disposable ppiDisposable = null;

    // plot functionality
    public Boolean apiConnected = Boolean.FALSE;

    private XYPlot plotHR, plotACC, plotPPG;
    private TimePlotterHR timeplotter;

    private PlotterACC timeplotterACC;
    private PlotterPPG timeplotterPPG;
    //private PlotterPPG timeplotter;

    public PlotterListener plotterListener = new PlotterListener() {
        @Override
        public void update() {
            plotHR.redraw();
            plotACC.redraw();
            plotPPG.redraw();
        }
    };

    // start streaming buttons
    public ToggleButton startAcc;
    public ToggleButton startPPG;
    public ToggleButton startPPI;

    // show plots
    public RadioGroup showPlots;
    public RadioButton hrPlot;
    public RadioButton accPlot;
    public RadioButton ppgPLot;

    // export acc/ppg/ppi
    public Button exportData;

    // generate acc/ppg/ppi csv data
    StringBuilder hrCSV = new StringBuilder();
    StringBuilder accCSV = new StringBuilder();
    StringBuilder ppgCSV = new StringBuilder();
    StringBuilder ppiCSV = new StringBuilder();

    // start recording button
    public ToggleButton start_stop_recording;
    public Boolean recording = false;


    // start streaming to lsl
    public LSLStream streamHr;
    public LSLStream streamAcc;
    public LSLStream streamPPG;
    public LSLStream streamPPI;

    // record device start time in nanoseconds
    public long start_time;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.polar_oh1_frag, container, false);
        sharedPreferences = this.getActivity().getPreferences(Context.MODE_PRIVATE);

        //Default to hide all plots at first
        plotHR = view.findViewById(R.id.plotHR);
        plotHR.setVisibility(View.GONE);
        plotACC = view.findViewById(R.id.plotACC);
        plotACC.setVisibility(View.GONE);
        plotPPG = view.findViewById(R.id.plotPPG);
        plotPPG.setVisibility(View.GONE);


        // Enter device ID text field
        EditText enterIdText = (EditText) view.findViewById(R.id.editTextSetID_frag2);
        enterIdText.setInputType(InputType.TYPE_CLASS_TEXT);
        enterIdText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
                if (sharedPreferences.getString(DEVICE_ID, null) != null) {
                    enterIdText.setHint(DEVICE_ID);
                }
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {  }

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
        toggle = (ToggleButton) view.findViewById(R.id.start_stop_connection_frag2);
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

        // Start and stop recording data
        start_stop_recording = (ToggleButton) view.findViewById(R.id.start_stop_recording_frag2);
        start_stop_recording.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    recording = true;
                } else {
                   recording = false;
                }
            }
        });

        // edit hrCSV file
        hrCSV.append("System Time,Internal Time,hr");

        // edit accCSV file
        accCSV.append("System Time,Internal Time,x,y,z");

        // start acc streaming
        startAcc = (ToggleButton) view.findViewById(R.id.start_acc_frag2);
        startAcc.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    startAcc.setChecked(false);
                }
                else {
                    //create ACC stream
                    streamAcc = new LSLStream();
                    // streaming LSL
                    // declare info strings to store stream data info for LSL
                    // info input in format of: { [0] "device name", [1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id"}
                    String deviceNameAcc = "PolarOH1/"+DEVICE_ID+"/ACC";
                    String[] accInfo = new String[]{deviceNameAcc, "ACC", "3", "50", DEVICE_ID};
                    try {
                        streamAcc.StreamOutlet(accInfo);
                    } catch (IOException e) {
                        e.printStackTrace();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }


                    if (isChecked) {
                        accelerometerData.setText("loading data...");
                        // test
                        if(accDisposable == null) {
                            accDisposable = api.requestAccSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarAccelerometerData>>) settings -> {
                                PolarSensorSetting sensorSetting = settings.maxSettings();
                                return api.startAccStreaming(DEVICE_ID, sensorSetting);
                            }).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    new Consumer<PolarAccelerometerData>() {
                                        @Override
                                        public void accept(PolarAccelerometerData polarAccData) throws Exception {
                                            if (recording) {
                                                for( PolarAccelerometerData.PolarAccelerometerSample data : polarAccData.samples ){
                                                    // get system data and time
                                                    SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                                                    sdf.setTimeZone(TimeZone.getDefault());
                                                    String currentDateAndTime = sdf.format(new Date());
                                                    accCSV.append("\n"+currentDateAndTime+","+ getElapsedNanoTime() + "," +data.x+","+data.y+","+data.z);
                                                }
                                            }

                                            accelerometerData.setText("    x: " + polarAccData.samples.get(0).x + "mG   y: " + polarAccData.samples.get(0).y + "mG   z: "+ polarAccData.samples.get(0).z + "mG");
                                            timeplotterACC.addValues(polarAccData.samples.get(0));
                                            streamAcc.runAcc(polarAccData.samples);
                                        }
                                    },

                                    new Consumer<Throwable>() {
                                        @Override
                                        public void accept(Throwable throwable) throws Exception {
                                            Log.e(TAG, "" + throwable.getLocalizedMessage());
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
                        } else {
                            // NOTE dispose will stop streaming if it is "running"
                            accDisposable.dispose();
                            accDisposable = null;
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
                        // if it is not checked, stop streaming acc data
                        if (streamAcc.outlet != null) {
                            streamAcc.close();
                        }
                    }
                }
            }
        });


        // edit ppg CSV
        ppgCSV.append("System Time,Internal Time,ppg0,ppg1,ppg2");

        // start ppg streaming
        startPPG = (ToggleButton) view.findViewById(R.id.start_ppg_frag2);
        startPPG.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    startPPG.setChecked(false);
                }
                else {

                    //create PPG stream
                    streamPPG = new LSLStream();
                    // streaming LSL
                    // declare info strings to store stream data info for LSL
                    // info input in format of: { [0] "device name", [1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id"}
                    String deviceNamePPG = "PolarOH1/"+DEVICE_ID+"/PPG";
                    String[] ppgInfo = new String[]{deviceNamePPG, "PPG", "3", "130", DEVICE_ID};
                    try {
                        streamPPG.StreamOutlet(ppgInfo);
                    } catch (IOException e) {
                        e.printStackTrace();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    if (isChecked) {
                        ppgData.setText("loading data...");
                        // test
                        if(ppgDisposable == null) {
                            ppgDisposable = api.requestPpgSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarOhrPPGData>>) polarPPGSettings -> api.startOhrPPGStreaming(DEVICE_ID,polarPPGSettings.maxSettings())).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    new Consumer<PolarOhrPPGData>() {
                                        @Override
                                        public void accept(PolarOhrPPGData polarPPGData) throws Exception {
                                            Log.d(TAG, "accelerometer update");
                                            if (recording) {
                                                for( PolarOhrPPGData.PolarOhrPPGSample data : polarPPGData.samples ){
                                                    SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                                                    sdf.setTimeZone(TimeZone.getDefault());
                                                    String currentDateAndTime = sdf.format(new Date());
                                                    ppgCSV.append("\n"+currentDateAndTime + "," + getElapsedNanoTime() + ","+data.ppg0+","+data.ppg1+","+data.ppg2);
                                                }
                                            }
                                            // start streaming ppg to lsl
                                            streamPPG.runPPG(polarPPGData.samples);
                                            float avg = (polarPPGData.samples.get(0).ppg0 + polarPPGData.samples.get(0).ppg1 + polarPPGData.samples.get(0).ppg2) / 3;
                                            ppgData.setText("ppg0: " + polarPPGData.samples.get(0).ppg0 + "   ppg1: " + polarPPGData.samples.get(0).ppg1 + "   ppg2: " + polarPPGData.samples.get(0).ppg2);
                                            timeplotterPPG.sendSingleSample((float) ((float) avg));
                                        }
                                    },
                                    new Consumer<Throwable>() {
                                        @Override
                                        public void accept(Throwable throwable) throws Exception {
                                            Log.e(TAG, "" + throwable.getLocalizedMessage());
                                            ppgDisposable = null;
                                        }
                                    },
                                    new Action() {
                                        @Override
                                        public void run() throws Exception {
                                            Log.d(TAG, "complete");
                                        }
                                    }
                            );
                        } else {
                            ppgDisposable.dispose();
                            ppgDisposable = null;
                        }

                        //show plot
                        showPlotPPG(view);
                        plotPPG.clear();
                    } else {
                        if (ppgDisposable != null) {
                            ppgDisposable.dispose();
                            ppgDisposable = null;
                        }
                        ppgData.setText("");
                        // if it is not checked, stop streaming ppg data
                        if (streamPPG.outlet != null) {
                            streamPPG.close();
                        }
                    }
                    plotPPG.setVisibility(View.GONE);
                }
            }
        });

        // edit ppi CSV
        ppiCSV.append("System Time,Internal Time,ppi");

        // start PPi streaming
        startPPI = (ToggleButton) view.findViewById(R.id.start_ppi_frag2);
        startPPI.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    startPPI.setChecked(false);
                }
                else {

                    //create PPI stream
                    streamPPI = new LSLStream();
                    // streaming LSL
                    // declare info strings to store stream data info for LSL
                    // info input in format of: { [0] "device name", [1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id"}
                    String deviceNamePPI = "PolarOH1/"+DEVICE_ID+"/PPI";
                    String[] ppiInfo = new String[]{deviceNamePPI, "PPI", "1", "130", DEVICE_ID};
                    try {
                        streamPPI.StreamOutlet(ppiInfo);
                    } catch (IOException e) {
                        e.printStackTrace();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }


                    if (isChecked) {
                        ppiData.setText("loading data...");
                        if(ppiDisposable == null) {
                            ppiDisposable = api.startOhrPPIStreaming(DEVICE_ID).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    polarOhrPPIData -> {
                                        if (recording) {
                                            for(PolarOhrPPIData.PolarOhrPPISample sample : polarOhrPPIData.samples) {
                                                SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                                                sdf.setTimeZone(TimeZone.getDefault());
                                                String currentDateAndTime = sdf.format(new Date());
                                                ppiCSV.append("\n"+currentDateAndTime+","+getElapsedNanoTime()+","+sample.ppi);
//                                                if (sample.hr != 0) {
//                                                    hrCSV.append("\n"+currentDateAndTime+","+getElapsedNanoTime() + ","+sample.hr);
//                                                }
                                            }
                                        }
                                        // display data in UI
                                        ppiData.setText(polarOhrPPIData.samples.get(0).ppi + "ms");

                                        //Toast.makeText(classContext, "in ppi", Toast.LENGTH_LONG).show();

                                        // start streaming ppi to lsl
                                        streamPPI.runPPI(polarOhrPPIData.samples);

                                    },
                                    throwable -> Log.e(TAG,""+throwable.getLocalizedMessage()),
                                    () -> Log.d(TAG,"complete")
                            );
                        } else {
                            ppiDisposable.dispose();
                            ppiDisposable = null;
                        }
                    } else {
                        if (ppiDisposable != null) {
                            ppiDisposable.dispose();
                            ppiDisposable = null;
                        }
                        ppiData.setText("");
                        // if it is not checked, stop streaming ppi data
                        if (streamPPI.outlet != null) {
                            streamPPI.close();
                        }
                    }
                }
            }
        });

        // set show plots radio buttons
        showPlots = (RadioGroup) view.findViewById(R.id.radioGroupShowPlots);
        hrPlot = (RadioButton) view.findViewById(R.id.hr_plot);
        accPlot = (RadioButton) view.findViewById(R.id.acc_plot);
        ppgPLot = (RadioButton) view.findViewById(R.id.ppg_plot);


        showPlots.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                if(checkedId == R.id.hr_plot) {
                    plotPPG.clear();
                    plotPPG.setVisibility(View.GONE);
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
                    showPlotHR(view);
                    plotHR.setVisibility(View.VISIBLE);
                } else if(checkedId == R.id.acc_plot) {
                    plotPPG.clear();
                    plotPPG.setVisibility(View.GONE);
                    plotHR.clear();
                    plotHR.setVisibility(View.GONE);
                    showPlotACC(view);
                    plotACC.setVisibility(View.VISIBLE);
                } else if(checkedId == R.id.ppg_plot) {
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
                    plotHR.clear();
                    plotHR.setVisibility(View.GONE);
                    showPlotPPG(view);
                } else {
                    plotHR.clear();
                    plotHR.setVisibility(View.GONE);
                    plotPPG.clear();
                    plotPPG.setVisibility(View.GONE);
                    plotACC.clear();
                    plotACC.setVisibility(View.GONE);
                }
            }
        });

        // set up properties
        classContext = this.getActivity().getApplicationContext();
        classActivity = this.getActivity();
        textViewBattery = (TextView) view.findViewById(R.id.battery_frag2);
        connectStatus = (TextView) view.findViewById(R.id.status_frag2);
        heartRate = (TextView) view.findViewById(R.id.hr_frag2);
        accelerometerData = (TextView) view.findViewById(R.id.acc_frag2);
        ppgData = (TextView) view.findViewById(R.id.ppg_frag2);
        ppiData = (TextView) view.findViewById(R.id.ppi_frag2);
        showStartTime = (Chronometer) view.findViewById(R.id.timer_frag2);
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


        // export data
        exportData = (Button) view.findViewById(R.id.export_data);
        exportData.setOnClickListener(v -> {
            showExportDialogue(view);
        });

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
                connectStatus.append("Connected\n");
                textViewBattery.setText("loading data...");
                heartRate.setText("loading data...");
                accelerometerData.setText("loading data...");
                ppgData.setText("loading data...");
                ppiData.setText("loading data...");
                apiConnected = Boolean.TRUE;
                start_time = System.nanoTime();
            }

            @Override
            public void deviceConnecting(PolarDeviceInfo polarDeviceInfo) {
            }

            @Override
            public void deviceDisconnected(PolarDeviceInfo s) {
                apiConnected = Boolean.FALSE;
                accelerometerData.setText("");
                ppgData.setText("");
                ppiData.setText("");
                // if device is not connected, stop streaming hr data
                if(streamHr.outlet != null) streamHr.close();
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
            public void biozFeatureReady(String s) {

            }

            @Override
            public void hrFeatureReady(String s) {
                Log.d(TAG, "HR Feature ready " + s);
                //Create HR stream if HR is ready
                streamHr = new LSLStream();
                String deviceNameHR = "PolarOH1/"+DEVICE_ID+"/HR";
                String[] hrInfo = new String[]{deviceNameHR, "HR", "1", "1", DEVICE_ID};
                try {
                    streamHr.StreamOutlet(hrInfo);
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
                textViewBattery.setText(msg + "\n");
            }

            @Override
            public void hrNotificationReceived(String s, PolarHrData polarHrData) {
                Log.d(TAG, "HR " + polarHrData.hr);
                heartRate.setText(String.valueOf(polarHrData.hr)+"bpm");
                timeplotter.addValues(polarHrData);

                // edit hrCSV
                if (recording) {
                    SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss", Locale.getDefault());
                    sdf.setTimeZone(TimeZone.getDefault());
                    String currentDateAndTime = sdf.format(new Date());
                    hrCSV.append("\n"+currentDateAndTime+","+getElapsedNanoTime() + ","+polarHrData.hr);
                }
                // stream hr to lsl
                try {
                    streamHr.runHr(polarHrData.hr);
                } catch (InterruptedException e) {
                    e.printStackTrace();
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
            // reset chronometer time
            showStartTime.setBase(SystemClock.elapsedRealtime());
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
            showStartTime.stop();
            showStartTime.setText("");
            connectStatus.append("Disconnected\n");
            heartRate.setText("");
            accelerometerData.setText("");
            ppgData.setText("");
            accDisposable = null;
            ppgDisposable = null;
            ppiDisposable = null;
            textViewBattery.setText("");
            connectStatus.setText("");
            Log.d(TAG, "finish");
        } catch (PolarInvalidArgument a){
            a.printStackTrace();
        }
        plotHR.setVisibility(View.GONE);
        plotPPG.setVisibility(View.GONE);
        plotACC.setVisibility(View.GONE);
    }

    public void showPlotHR(View view){
        plotHR.setVisibility(View.VISIBLE);
        // Plot HR/RR graph
        timeplotter = new TimePlotterHR(classContext, "HR/RR");
        timeplotter.setListener(plotterListener);
        plotHR.addSeries(timeplotter.getHrSeries(), timeplotter.getHrFormatter());
        //plotHR.addSeries(timeplotter.getRrSeries(), timeplotter.getRrFormatter());
        plotHR.setRangeBoundaries(50, 100,
                BoundaryMode.AUTO);
        plotHR.setDomainBoundaries(0, 360000,
                BoundaryMode.AUTO);
        // Left labels will increment by 2
        plotHR.setRangeStep(StepMode.SUBDIVIDE, 5);
        plotHR.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
        // Make left labels be an integer (no decimal places)
        plotHR.getGraph().getLineLabelStyle(XYGraphWidget.Edge.LEFT).
                setFormat(new DecimalFormat("#"));
        plotHR.getLegend().setVisible(true);
    }

    public void showPlotACC(View view){
        plotACC.setVisibility(View.VISIBLE);
        //Plot ACC graph
        timeplotterACC = new PlotterACC(classContext, "ACC");
        timeplotterACC.setListener(plotterListener);
        plotACC.addSeries(timeplotterACC.getAccXSeries(), timeplotterACC.getAccXFormatter());
        plotACC.addSeries(timeplotterACC.getAccYSeries(), timeplotterACC.getAccYFormatter());
        plotACC.addSeries(timeplotterACC.getAccZSeries(), timeplotterACC.getAccZFormatter());
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

    public void showPlotPPG(View view) {
        plotPPG.setVisibility(View.VISIBLE);
        timeplotterPPG = new PlotterPPG(classContext, "PPG");
        timeplotterPPG.setListener(plotterListener);
        plotPPG.addSeries(timeplotterPPG.getSeries(), timeplotterPPG.getFormatter());
        plotPPG.setRangeBoundaries(-500000, 500000, BoundaryMode.AUTO);
        plotPPG.setRangeStep(StepMode.INCREMENT_BY_FIT, 200000);
        plotPPG.setDomainBoundaries(0, 500, BoundaryMode.FIXED);
        plotPPG.setDomainStep(StepMode.INCREMENT_BY_VAL, 60000);
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

            // saving ppg file into device
            FileOutputStream outPpg = classActivity.openFileOutput("ppg data.csv", Context.MODE_PRIVATE);
            outPpg.write((ppgCSV.toString()).getBytes());
            outPpg.close();

            // get hr file location
            File fileLocationHr = new File(classContext.getFilesDir(), "hr data.csv");
            Uri pathHr = FileProvider.getUriForFile(classContext, "com.hri_physio.polarstreamer", fileLocationHr);

            // get acc file location
            File fileLocationAcc = new File(classActivity.getFilesDir(), "acc data.csv");
            Uri pathAcc = FileProvider.getUriForFile(classContext, "com.hri_physio.polarstreamer", fileLocationAcc);

            // get ppg file location
            File fileLocationPpg = new File(classActivity.getFilesDir(), "ppg data.csv");
            Uri pathPpg = FileProvider.getUriForFile(classContext, "com.hri_physio.polarstreamer", fileLocationPpg);

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
            uris.add(pathPpg);
            fileIntent.putExtra(Intent.EXTRA_STREAM, uris);
            startActivity(Intent.createChooser(fileIntent, "Send Polar OH1 device data"));

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
