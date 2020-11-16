package com.hri_physio.polarstreamer;

import android.content.Context;
import android.graphics.Color;

import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYSeriesFormatter;

import java.util.Arrays;

import polar.com.sdk.api.model.PolarAccelerometerData;

/**
 * Implements 3 series for Accelerometer XYZ using index-based x values.
 */

public class PlotterACC {

    String title;
    private String TAG = "Polar_Plotter";
    private PlotterListener listener;
    private Context context;
    private Number[] yaccXVals = new Number[300];
    private Number[] yaccYVals = new Number[300];
    private Number[] yaccZVals = new Number[300];
    private XYSeriesFormatter accXFormatter;
    private XYSeriesFormatter accYFormatter;
    private XYSeriesFormatter accZFormatter;
    private SimpleXYSeries accXSeries;
    private SimpleXYSeries accYSeries;
    private SimpleXYSeries accZSeries;
    private int index;

    public PlotterACC(Context context, String title) {
        this.context = context;
        this.title = title;  // Not used

        // Format series: ACC X/Y/Z
        accXFormatter = new LineAndPointFormatter(Color.RED,
                null, null, null);
        accXFormatter.setLegendIconEnabled(true);
        accXSeries = new SimpleXYSeries(Arrays.asList(yaccXVals),
                SimpleXYSeries.ArrayFormat.Y_VALS_ONLY,
                "X");

        accYFormatter = new LineAndPointFormatter(Color.BLUE,
                null, null, null);
        accYFormatter.setLegendIconEnabled(true);
        accYSeries = new SimpleXYSeries(Arrays.asList(yaccYVals),
                SimpleXYSeries.ArrayFormat.Y_VALS_ONLY,
                "Y");

        accZFormatter = new LineAndPointFormatter(Color.GREEN,
                null, null, null);
        accZFormatter.setLegendIconEnabled(true);
        accZSeries = new SimpleXYSeries(Arrays.asList(yaccZVals),
                SimpleXYSeries.ArrayFormat.Y_VALS_ONLY,
                "Z");
    }

    public SimpleXYSeries getAccXSeries() {
        return (SimpleXYSeries) accXSeries;
    }

    public SimpleXYSeries getAccYSeries() {
        return (SimpleXYSeries) accYSeries;
    }

    public SimpleXYSeries getAccZSeries() {
        return (SimpleXYSeries) accZSeries;
    }

    public XYSeriesFormatter getAccXFormatter() {
        return accXFormatter;
    }

    public XYSeriesFormatter getAccYFormatter() {
        return accYFormatter;
    }

    public XYSeriesFormatter getAccZFormatter() {
        return accZFormatter;
    }

    /**
     * Implements a strip chart by moving series data backwards and adding
     * new data at the end.
     *
     * @param polarAccData The ACC data that came in.
     */
    public void addValues(PolarAccelerometerData.PolarAccelerometerSample polarAccData) {
        yaccXVals[index] = polarAccData.x;
        yaccYVals[index] = polarAccData.y;
        yaccZVals[index] = polarAccData.z;
        if(index >= yaccXVals.length - 1){
            index = 0;
        }
        if(index < yaccXVals.length - 1){
            yaccXVals[index + 1] = null;
            yaccYVals[index + 1] = null;
            yaccZVals[index + 1] = null;
        }

        ((SimpleXYSeries) accXSeries).setModel(Arrays.asList(yaccXVals), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY);
        ((SimpleXYSeries) accYSeries).setModel(Arrays.asList(yaccYVals), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY);
        ((SimpleXYSeries) accZSeries).setModel(Arrays.asList(yaccZVals), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY);
        index++;
        listener.update();
    }

    public void setListener(PlotterListener listener) {
        this.listener = listener;
    }
}

