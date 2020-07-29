/*package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OdometryThread extends Thread {

    private DriveObject odo;
    private ValueStorage vals;
    private int[] dimensions;
    private int numWheels;
    private double lastTime = 0;

    private volatile boolean stop;

    public OdometryThread(int numWheels, DriveObject d, ValueStorage s, int[] dimensions) {
        this.odo = d;
        this.vals = s;
        switch (numWheels) {
            case 3:
                this.numWheels = 3;
                break;
            default:
                System.out.println("A " + numWheels + "-wheel odometry configuration is not supported.");
        }
        this.dimensions = dimensions;
    }

    public void run() {
        ElapsedTime time = new ElapsedTime();
        while(!stop) {
            if(time.milliseconds() - lastTime >= 5) {
                double[] encoderVals = vals.hardware(false, null)[odo.getPartNum()];
                switch (numWheels) {
                    case 3:
                        vals.runValues(true, threeWheel(encoderVals));
                }
            }
        }
    }

    public void Stop() {
        stop = true;
    }

    public double[] threeWheel(double[] encoderVals) {
        double x, y, heading;
        x = y = heading = 1; //Delete later
        //Logic to track position - probably gonna need a hand for this one, Antwon
        return new double[]{x, y, heading};
    }
}
*/