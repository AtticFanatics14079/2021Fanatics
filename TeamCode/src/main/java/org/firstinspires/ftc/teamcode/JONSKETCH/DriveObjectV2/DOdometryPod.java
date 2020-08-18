package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class DOdometryPod implements DriveObject{

    private DcMotorImplEx odoPod;
    private int partNum;
    private double ticksPerInch = 1000; //MODIFY WITH THE EXACT VALUE

    private ValueStorage vals;

    public void set(double value) {
        //Do nothing
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double value) {

    }

    public double[] getHardware() {
        return new double[]{odoPod.getCurrentPosition()};
    }

    public void endThreads() {
        //Do nothing
    }
}
