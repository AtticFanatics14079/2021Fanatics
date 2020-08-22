package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class DThreeWheelOdo implements Odometry, DriveObject {

    DOdometryPod[] odoPods;

    double x, y, heading, inchesPerTick;
    int partNum;

    //First val is trackwith, second value is forward offset, third is auxiliary trackwidth
    double[] dimensions;
    DOThread thread = new NullThread();

    ValueStorage vals;

    public DThreeWheelOdo(double x, double y, double heading, ValueStorage vals, DOdometryPod[] odoPods, double inchesPerTick, double... dimensions){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.odoPods = odoPods;
        this.vals = vals;
        this.dimensions = dimensions;
        this.inchesPerTick = inchesPerTick;
    }

    public void set(double value) {
        //Do nothing
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return new double[0];
    }

    public void setHardware(double value) {
        //Do nothing
    }

    public double[] getHardware() {
        return new double[0];
    }

    public void endThreads() {
        thread.Stop();
    }

    public Point getPosition() {
        return new Point(x, y, heading);
    }

    public void setPosition(Point newPos) {
        x = newPos.x;
        y = newPos.y;
        heading = newPos.heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void beginTracking() {
        thread = new OdometryThread(this, vals, dimensions, x, y, heading, inchesPerTick);
        thread.start();
    }
}
