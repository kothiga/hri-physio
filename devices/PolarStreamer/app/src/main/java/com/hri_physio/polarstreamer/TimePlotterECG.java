package com.hri_physio.polarstreamer;

import android.content.Context;
import android.graphics.Color;

import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYSeriesFormatter;

import java.util.Arrays;
import java.util.Date;

/**
 * Implements series ECG using time for the x values.
 */
public class TimePlotterECG {
    private static final int NVALS = 360;  // 360 sec

    String title;
    private String TAG = "Polar_Plotter";
    private PlotterListener listener;
    private Context context;
    private XYSeriesFormatter ecgFormatter;
    private SimpleXYSeries ecgSeries;
    private Double[] xEcgVals = new Double[NVALS];
    private Double[] yEcgVals = new Double[NVALS];

    public TimePlotterECG(Context context, String title) {
        this.context = context;
        this.title = title;  // Not used
        Date now = new Date();
        double endTime = now.getTime();
        double startTime = endTime - NVALS * 1000;
        double delta = (endTime - startTime) / (NVALS - 1);

        // Specify initial values to keep it from auto sizing
        for (int i = 0; i < NVALS; i++) {
            xEcgVals[i] = new Double(startTime + i * delta);
            yEcgVals[i] = new Double(0);
        }

        // Format series: ECG
        ecgFormatter = new LineAndPointFormatter(Color.RED,
                null, null, null);
        ecgSeries = new SimpleXYSeries(Arrays.asList(xEcgVals),
                Arrays.asList(yEcgVals),
                "ECG (mV)");
    }

    public SimpleXYSeries getEcgSeries() { return (SimpleXYSeries) ecgSeries; }

    public XYSeriesFormatter getEcgFormatter() {
        return ecgFormatter;
    }

    /**
     * Implements a strip chart by moving series data backwards and adding
     * new data at the end.
     *
     * @param ecgSample The single ECG data that came in.
     */
    public void addValues(float ecgSample) {
        Date now = new Date();
        long time = now.getTime();
        double endTime = now.getTime();
        double startTime = endTime - NVALS * 1000;
        for (int i = 0; i < NVALS - 1; i++) {
            xEcgVals[i] = xEcgVals[i + 1];
            yEcgVals[i] = yEcgVals[i + 1];
            ecgSeries.setXY(xEcgVals[i], yEcgVals[i], i);
        }
        xEcgVals[NVALS - 1] = new Double(time);
        yEcgVals[NVALS - 1] = new Double(ecgSample);
        ecgSeries.setXY(xEcgVals[NVALS - 1], yEcgVals[NVALS - 1], NVALS - 1);

        listener.update();
    }

    public void setListener(PlotterListener listener) {
        this.listener = listener;
    }
}

