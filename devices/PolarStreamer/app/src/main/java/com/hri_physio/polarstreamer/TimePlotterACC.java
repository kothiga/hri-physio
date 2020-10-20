package com.hri_physio.polarstreamer;

import android.content.Context;
import android.graphics.Color;

import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYSeriesFormatter;

import java.util.Arrays;
import java.util.Date;

import polar.com.sdk.api.model.PolarAccelerometerData;

/**
 * Implements 3 series for Acceloremeter XYZ using time for the x values.
 */
public class TimePlotterACC {
    private static final int NVALS = 120;  // 120 sec

    String title;
    private String TAG = "Polar_Plotter";
    private PlotterListener listener;
    private Context context;

    private XYSeriesFormatter accXFormatter;
    private XYSeriesFormatter accYFormatter;
    private XYSeriesFormatter accZFormatter;
    private SimpleXYSeries accXSeries;
    private SimpleXYSeries accYSeries;
    private SimpleXYSeries accZSeries;
    private Double[] xaccXVals = new Double[NVALS];
    private Double[] yaccXVals = new Double[NVALS];
    private Double[] xaccYVals = new Double[NVALS];
    private Double[] yaccYVals = new Double[NVALS];
    private Double[] xaccZVals = new Double[NVALS];
    private Double[] yaccZVals = new Double[NVALS];

    public TimePlotterACC(Context context, String title) {
        this.context = context;
        this.title = title;  // Not used
        Date now = new Date();
        double endTime = now.getTime();
        double startTime = endTime - NVALS * 1000;
        double delta = (endTime - startTime) / (NVALS - 1);

        // Specify initial values to keep it from auto sizing
        for (int i = 0; i < NVALS; i++) {
            //normal sitting position as initial
            xaccXVals[i] = new Double(startTime + i * delta);
            yaccXVals[i] = new Double(-80);
            xaccYVals[i] = new Double(startTime+ i * delta);
            yaccYVals[i] = new Double(-40);
            xaccZVals[i] = new Double(startTime+ i * delta);
            yaccZVals[i] = new Double(-900);
        }

        // Format series: ACC X/Y/Z
        accXFormatter = new LineAndPointFormatter(Color.RED,
                null, null, null);
        accXFormatter.setLegendIconEnabled(true);
        accXSeries = new SimpleXYSeries(Arrays.asList(xaccXVals),
                Arrays.asList(yaccXVals),
                "X");

        accYFormatter = new LineAndPointFormatter(Color.BLUE,
                null, null, null);
        accYFormatter.setLegendIconEnabled(true);
        accYSeries = new SimpleXYSeries(Arrays.asList(xaccYVals),
                Arrays.asList(yaccYVals),
                "Y");

        accZFormatter = new LineAndPointFormatter(Color.GREEN,
                null, null, null);
        accZFormatter.setLegendIconEnabled(true);
        accZSeries = new SimpleXYSeries(Arrays.asList(xaccZVals),
                Arrays.asList(yaccZVals),
                "Z");
    }

    public SimpleXYSeries getAccXSeries() {
        return (SimpleXYSeries) accXSeries;
    }

    public SimpleXYSeries getAccYSeries() { return (SimpleXYSeries) accYSeries; }

    public SimpleXYSeries getAccZSeries() {
        return (SimpleXYSeries) accZSeries;
    }

    public XYSeriesFormatter getAccXFormatter() {
        return accXFormatter;
    }

    public XYSeriesFormatter getAccYFormatter() {
        return accYFormatter;
    }

    public XYSeriesFormatter getAccZFormatter() { return accZFormatter; }

    /**
     * Implements a strip chart by moving series data backwards and adding
     * new data at the end.
     *
     * @param polarAccData The HR data that came in.
     */
    public void addValues(PolarAccelerometerData.PolarAccelerometerSample polarAccData) {

        Date now = new Date();
        long time = now.getTime();
        double endTime = now.getTime();
        double startTime = endTime - NVALS * 1000;

        // X series
        for (int i = 0; i < NVALS - 1; i++) {
            xaccXVals[i] = xaccXVals[i + 1];
            yaccXVals[i] = yaccXVals[i + 1];
            accXSeries.setXY(xaccXVals[i], yaccXVals[i], i);
        }
        xaccXVals[NVALS - 1] = new Double(time);
        yaccXVals[NVALS - 1] = new Double(polarAccData.x);
        accXSeries.setXY(xaccXVals[NVALS - 1], yaccXVals[NVALS - 1], NVALS - 1);

        // Y series
        for (int i = 0; i < NVALS - 1; i++) {
            xaccYVals[i] = xaccYVals[i + 1];
            yaccYVals[i] = yaccYVals[i + 1];
            accYSeries.setXY(xaccYVals[i], yaccYVals[i], i);
        }
        xaccYVals[NVALS - 1] = new Double(time);
        yaccYVals[NVALS - 1] = new Double(polarAccData.y);
        accYSeries.setXY(xaccYVals[NVALS - 1], yaccYVals[NVALS - 1], NVALS - 1);

        // Z series
        for (int i = 0; i < NVALS - 1; i++) {
            xaccZVals[i] = xaccZVals[i + 1];
            yaccZVals[i] = yaccZVals[i + 1];
            accZSeries.setXY(xaccZVals[i], yaccZVals[i], i);
        }
        xaccZVals[NVALS - 1] = new Double(time);
        yaccZVals[NVALS - 1] = new Double(polarAccData.z);
        accZSeries.setXY(xaccZVals[NVALS - 1], yaccZVals[NVALS - 1], NVALS - 1);

        listener.update();
    }

    public void setListener(PlotterListener listener) {
        this.listener = listener;
    }
}

