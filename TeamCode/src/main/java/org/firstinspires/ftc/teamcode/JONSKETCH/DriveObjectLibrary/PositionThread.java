package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

import android.util.Pair;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class PositionThread extends Thread {

    private int pos; //Arrays are all ordered according to partNum.
    private double lastTime, tolerance, error, totalError, lastError, maxSpeed;
    private volatile ArrayList<DriveObject> drive = new ArrayList<>();
    private volatile ArrayList<Double[]> PID = new ArrayList<>();
    private boolean stillGoing = true, group = false;
    private volatile boolean stop;

    public PositionThread(int position, double maxSpeed, double tolerance, DriveObject drive){
        this.drive.add(drive);
        pos = position;
        this.maxSpeed = maxSpeed;
        PID.add(drive.getPID().clone());
        this.tolerance = tolerance;
    }

    public PositionThread(int position, double maxSpeed, double tolerance, DriveObject[] drive){
        group = true;
        pos = position;
        for(DriveObject d : drive) {
            this.drive.add(d);
            this.maxSpeed = maxSpeed;
            PID.add(d.getPID().clone());
        }
        this.tolerance = tolerance;
    }

    public PositionThread(){}

    public void run(){
        ElapsedTime time = new ElapsedTime();
        lastTime = 0;
        double velocity;
        while(!stop && stillGoing) {
            if(time.milliseconds() - lastTime >= 5) {
                lastTime = time.milliseconds();
                stillGoing = false;
                if(group){
                    stillGoing = true;
                    velocity = groupToPosition();
                    if(Math.abs(error) < tolerance) {
                        velocity = 0;
                        stop = true;
                    }
                    for(DriveObject d : drive) {
                        d.setPower(velocity);
                    }
                }
                else if(drive.get(0) != null){
                    stillGoing = true;
                    velocity = toPosition();
                    drive.get(0).setPower(velocity);
                    if(Math.abs(error) < tolerance) { //This is using velocity as an indicator to stop, may change later.
                        drive.get(0).setPower(0);
                        drive.set(0, null);
                    }
                }
            }
        }
    }

    private double toPosition(){
        double velocity = 0;
        error = pos - drive.get(0).get();
        if(drive.get(0).getClassification() == DriveObject.classification.Drivetrain) {
            error = pos - drive.get(0).get(1);
        }
        totalError += error;
        velocity += PID.get(0)[0] * error;
        velocity += PID.get(0)[1] * totalError;
        velocity += PID.get(0)[2] * (error - lastError);

        if(velocity > PID.get(0)[3]) velocity = PID.get(0)[3];

        velocity *= maxSpeed;

        lastError = error;

        return velocity;
    }

    private double groupToPosition(){
        double velocity = 0;
        error = pos;
        for(DriveObject n : drive) error -= n.get()/4.0;
        totalError += error;
        velocity += PID.get(0)[0] * error;
        velocity += PID.get(0)[1] * totalError;
        velocity += PID.get(0)[2] * (error - lastError);

        if(Math.abs(velocity) > PID.get(0)[3]) velocity = PID.get(0)[3] * Math.abs(velocity) / velocity;

        velocity *= Math.abs(maxSpeed);

        lastError = error;

        return velocity;
    }

    public void Stop(){
        stop = true;
    }
}
