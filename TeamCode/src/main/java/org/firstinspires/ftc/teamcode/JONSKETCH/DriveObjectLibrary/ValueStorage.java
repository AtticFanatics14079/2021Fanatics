package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

import android.util.Pair;

import java.util.ArrayList;
import java.util.Arrays;

public class ValueStorage {

    private volatile double Time = 0.0; //In milliseconds

    public final int maxValues = 2; //Change to match the maximum number of variables a
    //DriveObject could take (e.g. if IMU takes 3 values and DcMotors take 4, set to 4).

    private Double[][] hardwareValues;

    private double[] runValues; //Same values as above, but used after algorithms
    //determine what the power should be.

    private Boolean[] changedParts; //Same as hardwareValues. Tells which parts
    // should be updated.

    public volatile boolean receivedDesiredVals = false;

    public void setup(int size){
        runValues = new double[size];
        hardwareValues = new Double[size][];
        changedParts = new Boolean[size];
        System.out.println(changedParts.length);
    }

    public void clear(){
        Arrays.fill(runValues, 0.0);
        Arrays.fill(hardwareValues, null);
        Arrays.fill(changedParts, null);
    }

    public synchronized Double[][] hardware(boolean writing, Double[][] values){
        if(writing) {
            for(int i = 0; i < values.length; i++) {
                hardwareValues[i] = values[i].clone();
            }
            return null;
        }
        return hardwareValues;
    }

    public synchronized Boolean[] changedParts(boolean Writing, Boolean[] desiredParts){
        if (Writing) {
            if(receivedDesiredVals) Arrays.fill(changedParts, false);
            for(int i = 0; i < desiredParts.length; i++) {
                if(desiredParts[i] != null) {
                    changedParts[i] = desiredParts[i];
                }
            }
            receivedDesiredVals = false;
            return null;
        }
        receivedDesiredVals = true;
        return changedParts;
    }

    public synchronized double[] runValues(boolean Writing, Double[] values){
        if(Writing) {
            for(int i = 0; i < values.length; i++) if(values[i] != null) runValues[i] = values[i];
            return null;
        }
        return runValues;
    }

    public synchronized double time(boolean writing, double time){
        if(writing){
            Time = time;
            return 0;
        }
        return Time;
    }
}