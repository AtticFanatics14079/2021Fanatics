package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

public class PositionThread extends Thread implements DOThread {

    private int pos; //Arrays are all ordered according to partNum.
    private double lastTime, tolerance, error, totalError, lastError, maxSpeed;
    private volatile DriveObject[] drive;
    private volatile double[] PID;
    private boolean stillGoing = true, group = false;
    private volatile boolean stop;

    public PositionThread(int position, double maxSpeed, double tolerance, DMotor motor){
        this.drive = new DriveObject[]{motor};
        pos = position;
        this.maxSpeed = maxSpeed;
        PID = motor.getPID().clone();
        this.tolerance = tolerance;
    }

    public PositionThread(int position, double maxSpeed, double tolerance, DMotor[] motors){
        group = true;
        pos = position;
        this.maxSpeed = maxSpeed;
        PID = motors[0].getPID().clone();
        this.drive = motors;
        this.tolerance = tolerance;
    }

    public PositionThread(){}

    public void run(){
        ElapsedTime time = new ElapsedTime();
        lastTime = 0;
        double velocity = 0;
        while(!stop && stillGoing) {
            if(time.milliseconds() - lastTime >= 5) {
                lastTime = time.milliseconds();
                stillGoing = false;
                if(group){
                    stillGoing = true;
                    try {
                        velocity = groupToPosition();
                    } catch(Exception e) {
                        System.out.println(e);
                    }
                    if(Math.abs(error) < tolerance) {
                        velocity = 0;
                        stop = true;
                    }
                    for(DriveObject d : drive) {
                        d.set(velocity);
                    }
                }
                else if(drive[0] != null){
                    stillGoing = true;
                    try {
                        velocity = toPosition();
                    } catch(Exception e) {
                        System.out.println(e);
                    }
                    drive[0].set(velocity);
                    if(Math.abs(error) < tolerance) { //This is using velocity as an indicator to stop, may change later.
                        drive[0].set(0);
                        drive[0] = null;
                    }
                }
            }
        }
    }

    private double toPosition(){
        double velocity = 0;
        error = pos - drive[0].get()[1];
        totalError += error;
        velocity += PID[0] * error;
        velocity += PID[1] * totalError;
        velocity += PID[2] * (error - lastError);

        if(velocity > PID[3]) velocity = PID[3] * Math.abs(velocity) / velocity;

        velocity *= maxSpeed;

        lastError = error;

        return velocity;
    }

    //Written with the assumption that all motors are "working together" so to speak and have the same PID (aka is intended for two-motor scissor-lifts,
    //two-motor intakes, drivetrain movements, etc.)
    private double groupToPosition(){
        double velocity = 0;
        error = pos;
        for(DriveObject n : drive) {
            error -= n.get()[1]/4.0;
        }
        totalError += error;
        velocity += PID[0] * error;
        velocity += PID[1] * totalError;
        velocity += PID[2] * (error - lastError);

        if(Math.abs(velocity) > PID[3]) velocity = PID[3] * Math.abs(velocity) / velocity;

        velocity *= Math.abs(maxSpeed);

        lastError = error;

        return velocity;
    }

    public void Stop(){
        stop = true;
    }
}
