package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.Geometry;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.Point;

import java.util.Arrays;

public class OdometryThread extends Thread {

    private DriveObject odo;
    private ValueStorage vals;
    private double[] dimensions; //First value is trackwidth, second value is forward offset, third is auxiliary trackwidth
    private int numWheels;
    private double lastTime = 0;
    private double[] lastPos;
    private double[] lastEncoderVals;
    private double inchesPerTick = 0.0001;
    private double x = 0, y = 0, heading = 0, degTraveled = 0;

    private volatile boolean stop;

    public OdometryThread(int numWheels, DriveObject d, ValueStorage s, double[] dimensions) {
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
        lastPos = lastEncoderVals = new double[]{0.0, 0.0, 0.0};
    }

    public OdometryThread(int numWheels, DriveObject d, ValueStorage s, double[] dimensions, int x, int y, double heading, int inchesPerTick) {
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
        lastEncoderVals = new double[]{0, 0, 0};
        lastPos = new double[]{x, y, heading};
        this.inchesPerTick = inchesPerTick;
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
        //Logic to track position - probably gonna need a hand for this one, Antwon
        double leftWheelChange = (encoderVals[0] - lastEncoderVals[0]) * inchesPerTick;
        double rightWheelChange = (encoderVals[1] - lastEncoderVals[1]) * inchesPerTick;
        double frontWheelChange = (encoderVals[2] - lastEncoderVals[2]) * inchesPerTick;
        double angleChange = (leftWheelChange - rightWheelChange) / dimensions[0]; //Brad (radians)
        double leftTotal = encoderVals[0] * inchesPerTick;
        double rightTotal = encoderVals[1] * inchesPerTick;
        double rawAngle = (leftTotal - rightTotal) / dimensions[0]; //Brad
        double auxiliaryPrediction = angleChange * dimensions[2];
        double forward = (leftWheelChange + rightWheelChange) / 2;
        double side = frontWheelChange - auxiliaryPrediction;

        //UpdatePos

        degTraveled += Math.toDegrees(angleChange); //Doug (aka degrees)
        double sineTerm = Math.sin(angleChange) / angleChange;
        double cosTerm = (1.0 - Math.cos(angleChange)) / angleChange;
        if(Math.abs(angleChange) < 0.000001) {
            sineTerm = 1.0 - angleChange * angleChange / 6.0;
            cosTerm = angleChange / 2.0;
        }
        double move = sineTerm * forward - cosTerm * side;
        double strafe = cosTerm * forward + sineTerm * side;
        y += move;
        x += strafe;
        heading = rawAngle;

        //Part of PointDelta

        /*double c = Math.cos(heading);
        double s = Math.sin(heading);
        double newY = move * c - strafe * s;
        double newX = move * s + strafe * c;
         */
        lastEncoderVals = encoderVals;
        lastPos[0] = x;
        lastPos[1] = y;
        lastPos[2] = heading;
        return new double[]{x, y, heading};
    }
}
