package com.hri_physio.polarstreamer;

import java.io.IOException;
import java.util.List;

import edu.ucsd.sccn.LSL;

public class LSLStream {
    public LSL.StreamOutlet outlet;
    public LSL.StreamInfo info;
    float[] chunk;
    public void StreamOutlet(String[] dataInfo) throws IOException, InterruptedException  {
        //Create new stream info
        // args in format of: { [0] "device name",[1]  "type of data", [2]"channel count", [3]"sampling rate", [4]"device id"}
        info = new LSL.StreamInfo(dataInfo[0],dataInfo[1],
                Integer.parseInt(dataInfo[2]),Integer.parseInt(dataInfo[3]),
                LSL.ChannelFormat.double64, dataInfo[4]);

        //Create outlet
        outlet = new LSL.StreamOutlet(info);
    }

    //currently not used
    public void run(int sample) throws InterruptedException {
        chunk = new float[2];
        for (int t=1;t<10;t++) { //sending a total of 10*6 chunks per stream
            // the chunk array contains first all values for the first sample, then the second, and so on
            for (int k=0;k<chunk.length;k++){
                chunk[k] = (float)sample;
            }
            outlet.push_chunk(chunk); // note: it is also possible to pass in time stamps
//            Thread.sleep(100);
        }
    }

    public void runHr(int sample) throws InterruptedException {
        chunk = new float[1];
        for (int k=0; k<chunk.length;k++){
            chunk[k] = (float)sample;
        }
        outlet.push_chunk(chunk);
    }

    public void runList(List<Integer> samples) throws InterruptedException {
        //Sending data by chunk size = list size
        chunk = new float[samples.size()];
        for (int k=0;k<chunk.length;k++){
            chunk[k] = (float)samples.get(k);
        }
        outlet.push_chunk(chunk);
    }

    public void close(){
        outlet.close();
        info.destroy();
    }
}
