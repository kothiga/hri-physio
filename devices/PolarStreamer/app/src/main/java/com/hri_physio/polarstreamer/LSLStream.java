package com.hri_physio.polarstreamer;

import java.io.IOException;
import java.util.List;

import edu.ucsd.sccn.LSL;
import polar.com.sdk.api.model.PolarAccelerometerData;
import polar.com.sdk.api.model.PolarOhrPPGData;
import polar.com.sdk.api.model.PolarOhrPPIData;

public class LSLStream {
    public LSL.StreamOutlet outlet;
    public LSL.StreamInfo info;
    public int[] chunk;
    public void StreamOutlet(String[] dataInfo) throws IOException, InterruptedException  {
        //Create new stream info
        // args in format of: { [0] "device name",[1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id"}
        info = new LSL.StreamInfo(dataInfo[0],dataInfo[1],
                Integer.parseInt(dataInfo[2]),Integer.parseInt(dataInfo[3]),
                LSL.ChannelFormat.double64, dataInfo[4]);

        //Create outlet
        outlet = new LSL.StreamOutlet(info);
    }

    public void runHr(int sample) throws InterruptedException {
        chunk = new int[]{sample};
        outlet.push_chunk(chunk);
    }

    public void runList(List<Integer> samples) throws InterruptedException {
        //Sending data by chunk size = list size
        chunk = new int[samples.size()];
        for (int k=0;k<chunk.length;k++){
            chunk[k] = samples.get(k);
        }
        outlet.push_chunk(chunk);
    }

    public void runPPI(List<PolarOhrPPIData.PolarOhrPPISample> samples) throws InterruptedException {
        chunk = new int[samples.size()];
        for (int k=0;k<chunk.length;k++){
            chunk[k] = samples.get(k).ppi;
        }
        outlet.push_chunk(chunk);
    }

    public void runAcc(List<PolarAccelerometerData.PolarAccelerometerSample> samples) throws InterruptedException {
        //Sending data by chunk size = list size
        chunk = new int[samples.size()*3];
        int sampleIndex = 0;
        for (int k=0;k<samples.size()*3;k+=3){
            chunk[k] = samples.get(sampleIndex).x;
            chunk[k+1] = samples.get(sampleIndex).y;
            chunk[k+2] = samples.get(sampleIndex).z;
            sampleIndex++;
        }
        outlet.push_chunk(chunk);
    }

    public void runPPG (List<PolarOhrPPGData.PolarOhrPPGSample> samples ) throws InterruptedException {
        chunk = new int[samples.size()*3];
        int sampleIndex = 0;
        for (int k=0;k<samples.size()*3;k+=3){
            chunk[k] = samples.get(sampleIndex).ppg0;
            chunk[k+1] = samples.get(sampleIndex).ppg1;
            chunk[k+2] = samples.get(sampleIndex).ppg2;
            sampleIndex++;
        }
        outlet.push_chunk(chunk);
    }

    public void close(){
        outlet.close();
        info.destroy();
    }
}
