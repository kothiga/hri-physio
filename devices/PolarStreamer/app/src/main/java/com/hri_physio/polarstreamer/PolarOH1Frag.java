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
import java.math.BigDecimal;
import java.text.DecimalFormat;
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
    private Plotter timeplotterPPG;
    public PlotterListener plotterListener = new PlotterListener() {
        @Override
        public void update() {
            plotHR.redraw();
            plotACC.redraw();
            plotPPG.redraw();
        }
    };

    //test
    public ToggleButton togglePlotACC;
    public ToggleButton togglePlotPPG;
    public ToggleButton startAcc;
    public ToggleButton startPPG;
    public ToggleButton startPPI;

    // show plots
    public RadioGroup showPlots;
    public RadioButton hrPlot;
    public RadioButton accPlot;
    public RadioButton ppgPLot;

    // export acc/ppg/ppi
    public Button exportAcc;
    public Button exportPpg;
    public Button exportPpi;

    // generate acc/ppg/ppi csv data
    StringBuilder accCSV = new StringBuilder();
    StringBuilder ppgCSV = new StringBuilder();
    StringBuilder ppiCSV = new StringBuilder();

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

        // edit accCSV file
        accCSV.append("Timestamp, x, y, z");

        // start acc streaming
        togglePlotACC = (ToggleButton) view.findViewById(R.id.start_acc_frag2);
        togglePlotACC.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    togglePlotACC.setChecked(false);
                }
                else {
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
                                            Log.d(TAG, "accelerometer update");
                                            accelerometerData.setText("    x: " + polarAccData.samples.get(0).x + "mG   y: " + polarAccData.samples.get(0).y + "mG   z: "+ polarAccData.samples.get(0).z + "mG");
                                            timeplotterACC.addValues(polarAccData.samples.get(0));
                                            String val = new BigDecimal(polarAccData.timeStamp).toPlainString();
                                            accCSV.append("\n"+"#"+val+","+polarAccData.samples.get(0).x+","+polarAccData.samples.get(0).y+","+polarAccData.samples.get(0).z);
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
                    }
                }
            }
        });


        // edit ppg CSV
        ppgCSV.append("Timestamp, ppg0, ppg1, ppg2");

        // start ppg streaming
        togglePlotPPG = (ToggleButton) view.findViewById(R.id.start_ppg_frag2);
        togglePlotPPG.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(!apiConnected){
                    Snackbar.make(view, "Device is not connected. Please start device connection to view plot. ", Snackbar.LENGTH_LONG)
                            .setAction("Action", null).show();
                    togglePlotPPG.setChecked(false);
                }
                else {
                    if (isChecked) {

                        // test
                        if(ppgDisposable == null) {
                            ppgDisposable = api.requestPpgSettings(DEVICE_ID).toFlowable().flatMap((Function<PolarSensorSetting, Publisher<PolarOhrPPGData>>) polarPPGSettings -> api.startOhrPPGStreaming(DEVICE_ID,polarPPGSettings.maxSettings())).subscribeOn(Schedulers.newThread()).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    new Consumer<PolarOhrPPGData>() {
                                        @Override
                                        public void accept(PolarOhrPPGData polarPPGData) throws Exception {
                                            Log.d(TAG, "accelerometer update");
                                            float avg = (polarPPGData.samples.get(0).ppg0 + polarPPGData.samples.get(0).ppg1 + polarPPGData.samples.get(0).ppg2) / 3;
                                            ppgData.setText("ppg0: " + polarPPGData.samples.get(0).ppg0 + "   ppg1: " + polarPPGData.samples.get(0).ppg1 + "   ppg2: " + polarPPGData.samples.get(0).ppg2);
                                            timeplotterPPG.sendSingleSample((float) ((float) avg));
                                            ppgCSV.append("\n"+"#"+polarPPGData.timeStamp+","+polarPPGData.samples.get(0).ppg0+","+polarPPGData.samples.get(0).ppg1+","+polarPPGData.samples.get(0).ppg2);
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
                        plotPPG.setVisibility(View.GONE);
                    } else {
                        ppgData.setText("");
                    }
                }
            }
        });

        // edit ppi CSV
        ppiCSV.append("Timestamp, ppi");

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
                    if (isChecked) {
                        if(ppiDisposable == null) {
                            ppiDisposable = api.startOhrPPIStreaming(DEVICE_ID).observeOn(AndroidSchedulers.mainThread()).subscribe(
                                    polarOhrPPIData -> {
                                        // display data in UI
                                        ppiData.setText(polarOhrPPIData.samples.get(0).ppi + "ms");
                                        ppiCSV.append("\n"+"#"+polarOhrPPIData.timeStamp+","+polarOhrPPIData.samples.get(0).ppi);
                                    },
                                    throwable -> Log.e(TAG,""+throwable.getLocalizedMessage()),
                                    () -> Log.d(TAG,"complete")
                            );
                        } else {
                            ppiDisposable.dispose();
                            ppiDisposable = null;
                        }
                    } else {
                        ppiData.setText("");
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


        // export acc
        exportAcc = (Button) view.findViewById(R.id.export_acc);
        exportAcc.setOnClickListener(v -> {
            showExportAccDialogue(view);
        });
        // export ppg
        exportPpg = (Button) view.findViewById(R.id.export_ppg);
        exportPpg.setOnClickListener(v-> {
            showExportPpgDialogue(view);
        });
        // export ppi
        exportPpi = (Button) view.findViewById(R.id.export_ppi);
        exportPpi.setOnClickListener(v-> {
            showExportPpiDialogue(view);
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
            }

            @Override
            public void deviceConnecting(PolarDeviceInfo polarDeviceInfo) {
            }

            @Override
            public void deviceDisconnected(PolarDeviceInfo s) {
                apiConnected = Boolean.FALSE;
            }

            @Override
            public void ecgFeatureReady(String s) {
                Log.d(TAG, "ECG Feature ready " + s);
            }

            @Override
            public void accelerometerFeatureReady(String s) {
                Log.d(TAG, "ACC Feature ready " + s);
                //Toast.makeText(classContext, "acc ready", Toast.LENGTH_LONG).show();
                //Snackbar.make(view, "acc ready", Snackbar.LENGTH_LONG).setAction("Action", null).show();
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
        //plotHR.setVisibility(View.GONE);
        //togglePlotACC.setChecked(false);
        //plotACC.setVisibility(View.GONE);
    }

    public void showPlotHR(View view){
        plotHR.setVisibility(View.VISIBLE);
        // Plot HR/RR graph
        timeplotter = new TimePlotterHR(classContext, "HR/RR");
        timeplotter.setListener(plotterListener);
        plotHR.addSeries(timeplotter.getHrSeries(), timeplotter.getHrFormatter());
        plotHR.addSeries(timeplotter.getRrSeries(), timeplotter.getRrFormatter());
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

    public void showPlotPPG(View view) {
        plotPPG.setVisibility(View.VISIBLE);
        timeplotterPPG = new Plotter(classContext, "PPG");
        timeplotterPPG.setListener(plotterListener);
        plotPPG.addSeries(timeplotterPPG.getSeries(), timeplotterPPG.getFormatter());
        plotPPG.setRangeBoundaries(200000, 500000, BoundaryMode.AUTO);
        plotPPG.setRangeStep(StepMode.INCREMENT_BY_FIT, 50000);
        plotPPG.setDomainBoundaries(0, 500, BoundaryMode.AUTO);
        plotPPG.setLinesPerRangeLabel(2);

    }

    public void showExportAccDialogue(View view) {
        try{
            //saving the file into device
            FileOutputStream out = classActivity.openFileOutput("acc data.csv", Context.MODE_PRIVATE);
            out.write((accCSV.toString()).getBytes());
            out.close();

            //exporting
            Context context = classContext;
            File fileLocation = new File(classActivity.getFilesDir(), "acc data.csv");
            Uri path = FileProvider.getUriForFile(context, "com.hri_physio.polarstreamer", fileLocation);
            Intent fileIntent = new Intent(Intent.ACTION_SEND);
            fileIntent.setType("text/csv");
            fileIntent.putExtra(Intent.EXTRA_SUBJECT, "Data");
            fileIntent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
            fileIntent.putExtra(Intent.EXTRA_STREAM, path);
            startActivity(Intent.createChooser(fileIntent, "Send Acc data"));
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void showExportPpgDialogue(View view) {
        try{
            //saving the file into device
            FileOutputStream out = classActivity.openFileOutput("ppg data.csv", Context.MODE_PRIVATE);
            out.write((ppgCSV.toString()).getBytes());
            out.close();

            //exporting
            Context context = classContext;
            File fileLocation = new File(classActivity.getFilesDir(), "ppg data.csv");
            Uri path = FileProvider.getUriForFile(context, "com.hri_physio.polarstreamer", fileLocation);
            Intent fileIntent = new Intent(Intent.ACTION_SEND);
            fileIntent.setType("text/csv");
            fileIntent.putExtra(Intent.EXTRA_SUBJECT, "Data");
            fileIntent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
            fileIntent.putExtra(Intent.EXTRA_STREAM, path);
            startActivity(Intent.createChooser(fileIntent, "Send Ppg data"));
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void showExportPpiDialogue(View view) {
//        try{
//            //saving the file into device
//            FileOutputStream out = classActivity.openFileOutput("ppi data.csv", Context.MODE_PRIVATE);
//            out.write((ppiCSV.toString()).getBytes());
//            out.close();
//
//            //exporting
//            Context context = classContext;
//            File fileLocation = new File(classActivity.getFilesDir(), "ppi data.csv");
//            Uri path = FileProvider.getUriForFile(context, "com.hri_physio.polarstreamer", fileLocation);
//            Intent fileIntent = new Intent(Intent.ACTION_SEND);
//            fileIntent.setType("text/csv");
//            fileIntent.putExtra(Intent.EXTRA_SUBJECT, "Data");
//            fileIntent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
//            fileIntent.putExtra(Intent.EXTRA_STREAM, path);
//            startActivity(Intent.createChooser(fileIntent, "Send ppi data"));
//        }
//        catch(Exception e){
//            e.printStackTrace();
//        }
    }
}
