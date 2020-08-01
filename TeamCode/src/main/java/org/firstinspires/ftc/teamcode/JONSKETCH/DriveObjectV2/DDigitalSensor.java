package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class DDigitalSensor implements DriveObject{

    private TouchSensor sensor;
    private int partNum;

    //Maybe add a thread to stop when a condition is met
    //private DOThread thread = new NullThread();
    private ValueStorage vals;

    //Constructors

    public DDigitalSensor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum) {
        sensor = hwMap.get(TouchSensor.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
    }

    //Interface methods

    public void set(double value) {
        //Do nothing
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double value) {
        //Do nothing
    }

    public double[] getHardware() {
        return new double[]{sensor.getValue()};
    }

    public void endThreads() {
        //thread.Stop();
    }
}
