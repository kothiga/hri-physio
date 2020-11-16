package com.hri_physio.polarstreamer;

import android.content.Context;
import android.graphics.Color;

import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYSeriesFormatter;

import java.util.Arrays;
import java.util.List;

/**
 * Implements series ECG using index based x values.
 */
public class PlotterECG {
    String title;
    private String TAG = "Polar_Plotter";
    private PlotterListener listener;
    private Context context;
    private Number[] yEcgVals = new Number[500];
    private XYSeriesFormatter ecgFormatter;
    private SimpleXYSeries ecgSeries;
    private int index;


    public PlotterECG(Context context, String title){
        this.context = context;
        this.title = title;

        for(int i = 0; i < yEcgVals.length - 1; i++){
            yEcgVals[i] = 0;
        }

        ecgFormatter = new LineAndPointFormatter(Color.RED,
                null, null, null);
        ecgSeries = new SimpleXYSeries(Arrays.asList(yEcgVals), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY,
                "ECG (mV)");
    }

    public SimpleXYSeries getSeries(){
        return (SimpleXYSeries) ecgSeries;
    }

    public XYSeriesFormatter getFormatter(){
        return ecgFormatter;
    }

    public void sendList(List<Integer> samples){
        int size = samples.size();

        for (int i = 0; i < size; i++) {
            if(i < yEcgVals.length - 1){
                yEcgVals[index+1] = null;
            }
            yEcgVals[index] = (float) ((float) samples.get(i) / 1000.0);
            index++;
            if(index >= yEcgVals.length - 1){
                index = 0;
                break;
            }
        }
        ((SimpleXYSeries) ecgSeries).setModel(Arrays.asList(yEcgVals), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY);
        listener.update();
    }

    public void setListener(PlotterListener listener){
        this.listener = listener;
    }
}

