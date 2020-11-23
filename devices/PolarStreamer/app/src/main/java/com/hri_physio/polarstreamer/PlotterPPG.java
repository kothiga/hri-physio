package com.hri_physio.polarstreamer;

import android.content.Context;
import android.graphics.Color;
import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYSeriesFormatter;
import java.util.Arrays;

public class PlotterPPG {
    String title;
    private String TAG = "Polar_Plotter";
    private PlotterListener listener;
    private Context context;
    private Number[] yPPGVal = new Number[800];
    private XYSeriesFormatter PPGFormatter;
    private SimpleXYSeries PPGSeries;
    private int index;

    public PlotterPPG(Context context, String title){
        this.context = context;
        this.title = title;

        PPGFormatter = new LineAndPointFormatter(Color.RED,
                null, null, null);
        PPGSeries = new SimpleXYSeries(Arrays.asList(yPPGVal), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY,
                "PPG");
    }

    public SimpleXYSeries getSeries(){
        return (SimpleXYSeries) PPGSeries;
    }

    public XYSeriesFormatter getFormatter(){
        return PPGFormatter;
    }

    public void sendSingleSample(float mV){
        yPPGVal[index] = mV;
        if(index >= yPPGVal.length - 1){
            index = 0;
        }
        if(index < yPPGVal.length - 1){
            yPPGVal[index + 1] = null;
        }

        ((SimpleXYSeries) PPGSeries).setModel(Arrays.asList(yPPGVal), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY);
        index++;
        listener.update();
    }

    public void setListener(PlotterListener listener){
        this.listener = listener;
    }
}
