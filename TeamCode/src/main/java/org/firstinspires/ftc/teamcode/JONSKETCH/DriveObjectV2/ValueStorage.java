package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import java.util.Arrays;

public class ValueStorage {

    //private volatile double Time = 0.0; //In milliseconds

    private double[][] hardwareValues; //Values returned by hardware

    private double[] runValues; //Values to for the robot to be set to (e.g. velocity for a DMotor).

    public void setup(int size){
        runValues = new double[size];
        hardwareValues = new double[size][];
    }

    public void clear(){
        Arrays.fill(runValues, 0.0);
        Arrays.fill(hardwareValues, null);
    }

    //Branched programming to allow for synchronous access to hardwareValues
    public synchronized double[] hardware(boolean writing, double[][] values, int partNum){
        if(writing) {
            for(int i = 0; i < values.length; i++) hardwareValues[i] = values[i].clone();
            return null;
        }

        //Returning clone to prevent threads changing/using the same value at the same time - potentially unnecessary
        return hardwareValues[partNum].clone();
    }

    public synchronized double[] runValues(boolean Writing, double value, int partNum){
        if(Writing) {
            runValues[partNum] = value;
            return null;
        }

        //Returning clone to prevent threads changing/using the same value at the same time - potentially unnecessary
        return runValues.clone();
    }

    //Used to be used
    /*public synchronized double time(boolean writing, double time){
        if(writing){
            Time = time;
            return 0;
        }
        return Time;
    }
     */
}