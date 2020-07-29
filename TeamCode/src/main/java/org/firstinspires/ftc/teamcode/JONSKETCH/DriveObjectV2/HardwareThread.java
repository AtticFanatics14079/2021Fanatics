package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;


public class HardwareThread extends Thread {

    ElapsedTime time;
    private ValueStorage vals;
    double[][] hardwareVals; //Holds the values received from hardware of each part.
    double[] lastRun; //Previous run values.
    public Configuration config;
    private volatile boolean stop;
    private boolean setTime = false;
    public double voltMult = 1, lastTime = 0;

    public HardwareThread(HardwareMap hwMap, ValueStorage vals, Configuration configuration){
        config = configuration;
        config.Configure(hwMap, vals);
        int size = config.hardware.size();
        this.vals = vals;
        this.vals.setup(size);
        hardwareVals = new double[size][];
        lastRun = new double[size];
        //voltMult = 13.0/config.voltSense.getVoltage();
        config.setBulkCachingManual();
    }

    public void run(){
        while(!setTime && !stop){}
        while(!stop) {
            if(time.milliseconds() - lastTime >= 5) {
                lastTime = time.milliseconds();

                //Used to be used
                //vals.time(true, time.milliseconds());

                readHardware();
                runHardware(vals.runValues(false, 0, 0));
            }
        }
        for(DriveObject d : config.hardware) {
            d.endThreads();
            vals.clear();
        }
    }

    public void startTime(ElapsedTime time){
        lastTime = time.milliseconds() - 2.5; //Sets lastTime off by the half interval
        this.time = time;
        setTime = true;
    }

    private void readHardware(){

        config.clearBulkCache();

        for(int i = 0; i < hardwareVals.length; i++) {
            hardwareVals[i] = config.hardware.get(i).getHardware();
        }

        vals.hardware(true, hardwareVals, 0);
    }

    private void runHardware(double[] Values) {

        for(int i = 0; i < this.hardwareVals.length; i++) {
            if(lastRun[i] != Values[i])
                config.hardware.get(i).setHardware(Values[i]);
        }

        lastRun = Values;
    }

    public void Stop(){
        stop = true;
    }
}
