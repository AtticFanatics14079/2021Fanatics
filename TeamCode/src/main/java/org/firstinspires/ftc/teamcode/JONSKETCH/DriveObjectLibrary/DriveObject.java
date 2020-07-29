package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;



import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;

public class DriveObject {

    private DcMotorImplEx motor;
    private Servo servo;
    private CRServo crservo;

    private DcMotorImplEx[] odometryWheels;

    private BNO055IMU imu;
    private TouchSensor touch;

    private int partNum;

    private double[] pid = {30.0, 0.0, 0.0, 3000.0}; //Default values

    private double[] p = new double[partNum + 1];

    private PositionThread posThread;
    private TimeThread timeThread;
    private OdometryThread odoThread;

    private static ValueStorage vals;
    private static HardwareMap hwMap;

    private String objectName;

    public enum type{
        DcMotorImplEx, Servo, CRServo, IMU, TouchSensor, Odometry, Null
    }

    private type thisType;

    public DriveObject(type typeName, String objectName, ValueStorage vals, int partNum, HardwareMap ahwMap){

        this.objectName = objectName;

        this.vals = vals;

        this.hwMap = ahwMap;

        this.partNum = partNum;

        thisType = typeName;

        switch(thisType){
            case DcMotorImplEx:
                motor = hwMap.get(DcMotorImplEx.class, objectName);
                break;
            case Servo:
                servo = hwMap.get(Servo.class, objectName);
                break;
            case CRServo:
                crservo = hwMap.get(CRServo.class, objectName);
                break;
            case IMU:
                imu = hwMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
                //Add configuration of IMU here
                break;
            case TouchSensor:
                touch = hwMap.get(TouchSensor.class, objectName);
                break;
            //For odometry, letting the motors be set later.
            case Odometry:
                break;
        }

        for(int i = 0; i < partNum; i++) p[i] = -100000;
    }

    //SECTION 1 : Get Methods for Private Variable Values

    public type getType(){
        return thisType;
    }

    public String getName(){
        return objectName;
    }

    public int getPartNum(){return partNum;}

    //SECTION 2 : Set Methods for Private Variables

    public void setType(type t){
        thisType = t;
    }

    public void setName(String n){
        objectName = n;
    }

    public void setPartNum(int p){partNum = p;}

    //SECTION 3 : Getting Hardware Values

    public double[] getHardware(){
        switch(thisType){
            case DcMotorImplEx:
                return new double[]{motor.getVelocity(), (double) motor.getCurrentPosition()};
            case Servo:
                return new double[]{servo.getPosition()}; //Fill in null for all values past usable and before reaching maxValues in ValueStorage
            case CRServo:
                return new double[]{crservo.getPower()};
            case IMU:
                return new double[]{(double) imu.getAngularOrientation().firstAngle, (double) imu.getAngularOrientation().secondAngle}; //Can add more returns here
            case TouchSensor:
                return new double[]{touch.getValue()};
            case Odometry:
                double[] vals = new double[odometryWheels.length + 1];
                for(int i = 1; i < odometryWheels.length; i++) vals[i] = (double) odometryWheels[i].getCurrentPosition();
                return vals; //May be able to return velocity here, not sure if rolling works for velocity.
            default:
                System.out.println("Invalid type, returning null.");
                return null;
        }
    }

    public double get(){
        return vals.hardware(false, null)[partNum][0];
    }

    public double get(int VariableNum){
        return vals.hardware(false, null)[partNum][VariableNum];
    }

    public double[] getAllVals(){
        return vals.hardware(false, null)[partNum];
    }

    //SECTION 4 : Setting Hardware Values

    public void setHardware(double Value){
        switch(thisType){
            case DcMotorImplEx:
                motor.setVelocity(Value);
                break;
            case Servo:
                servo.setPosition(Value);
                break;
            case CRServo:
                crservo.setPower(Value);
                break;
            default:
                break;
        }
    }

    public void set(double Value){
        Boolean[] b = new Boolean[partNum + 1];
        for(int i = 0; i <= partNum; i++) {
            b[i] = null;
        }
        switch(thisType){
            case DcMotorImplEx:
                p[partNum] = Value;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                //Maybe apply something to limit acceleration, but for now, just set power.
                break;
            case Servo:
                p[partNum] = Value;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                break;
            case CRServo:
                p[partNum] = Value;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                //See above for further changes.
                break;
            default:
                System.out.println("Invalid type for setting.");
                break;
        }
    }

    public void setPower(double Power){
        Boolean[] b = new Boolean[partNum + 1];
        for(int i = 0; i <= partNum; i++) {
            b[i] = null;
        }
        p[partNum] = Power;
        b[partNum] = true;
        vals.changedParts(true, b);
        vals.runValues(true, p);
    }

    //SECTION 5 : Adjusting Hardware (e.g. reversing motors)
    //DO NOT CALL THESE METHODS DURING THE MAIN BODY OF CODE, AS IT MAY STOP THE HARDWARE THREAD
    //Note: Potentially move these into the hardware thread and add respective values in ValueStorage later.

    public void reverse(){
        switch(thisType){
            case DcMotorImplEx:
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case CRServo:
                crservo.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case Servo:
                servo.setDirection(Servo.Direction.REVERSE);
                break;
            default:
                System.out.println("Invalid type for setting to reverse.");
        }
    }

    //RUN_TO_POSITION not supported, use setTargetPosition instead.
    public void setMode(DcMotor.RunMode mode){
        if(thisType == type.DcMotorImplEx && mode != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        if(thisType == type.DcMotorImplEx) motor.setZeroPowerBehavior(behavior);
    }

    //SECTION 6 : Thread Shenanigans

    public Thread setPower(double Power, double Seconds){
        if(thisType != type.DcMotorImplEx && thisType != type.CRServo) return null;
        timeThread = new TimeThread(Power, Seconds, this);
        //NOTE: If a TimeThread is called twice on the same motor, the first will not be overridden. May change this later.
        timeThread.start();
        return timeThread;
    }

    public Thread setTargetPosition(double targetPosition, double maxSpeed){
        switch(thisType){
            case Servo:
                Boolean[] b = new Boolean[partNum + 1];
                for(int i = 0; i < partNum; i++) {
                    b[i] = null;
                }
                p[partNum] = targetPosition;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                return null;
            case DcMotorImplEx:
                //if(pos.isAlive()) pos.stopPart(partNum); //Currently starting a new thread breaks a part
                if(posThread != null && posThread.isAlive()) posThread.Stop();
                posThread = new PositionThread((int) targetPosition, maxSpeed, 50, this);
                posThread.start();
                return posThread;
        }
        return null;
    }

    public Thread setTargetPosition(double targetPosition, double tolerance, double maxSpeed){
        switch(thisType){
            case Servo:
                Boolean[] b = new Boolean[partNum + 1];
                for(int i = 0; i < partNum; i++) {
                    b[i] = null;
                }
                p[partNum] = targetPosition;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                return null;
            case DcMotorImplEx:
                //if(pos.isAlive()) pos.stopPart(partNum); //Currently starting a new thread breaks a part
                if(posThread != null && posThread.isAlive()) posThread.Stop();
                posThread = new PositionThread((int) targetPosition, maxSpeed, tolerance, this);
                posThread.start();
                return posThread;
        }
        return null;
    }

    public Thread groupSetTargetPosition(int targetPos, double maxSpeed, double tolerance, DriveObject ...drive){
        boolean isIn = false;
        DriveObject[] drive2 = new DriveObject[drive.length + 1];
        for(int i = 0; i < drive.length; i++) {
            DriveObject d = drive[i];
            //if(d.getClassification() != classification.toPosition) return null;
            //Switch this later
            if(d == this) isIn = true;
            drive2[i] = d;
        }
        if(!isIn) drive2[drive.length] = this;
        //if(pos.isAlive()) pos.stopPart(partNum); //Currently starting a new thread breaks a part
        if(posThread != null && posThread.isAlive()) posThread.Stop();
        posThread = new PositionThread(targetPos, maxSpeed, tolerance, drive2);
        posThread.start();
        return posThread;
    }

    public Thread groupSetTargetPosition(int targetPos, double maxSpeed, double tolerance, ArrayList<DriveObject> drive){
        boolean isIn = false;
        for(DriveObject d : drive) {
            //if(d.getClassification() != classification.toPosition) return null;
            if(d == this) isIn = true;
        }
        if(!isIn) drive.add(this);
        if(posThread != null && posThread.isAlive()) posThread.Stop();
        DriveObject[] d = new DriveObject[drive.size()];
        int i = 0;
        for(DriveObject a : drive) d[i++] = a;
        posThread = new PositionThread(targetPos, maxSpeed, tolerance, d);
        posThread.start();
        return posThread;
    }

    public void endAllThreads(){
        if(posThread != null && posThread.isAlive()) posThread.Stop();
        if(timeThread != null && timeThread.isAlive()) timeThread.stop(); //Have to use this because timeThread is sleeping. May change later.
        if(odoThread != null && odoThread.isAlive()) odoThread.Stop();
    }

    //SECTION 7 : Algorithm Variables (e.g. PID)

    public double[] getPID(){
        return pid;
    }

    public void setPID(double p, double i, double d){
        pid[0] = p;
        pid[1] = i;
        pid[2] = d;
    }
    public void setPID(double p, double i, double d, double maxVelocity){
        pid[0] = p;
        pid[1] = i;
        pid[2] = d;
        pid[3] = maxVelocity;
    }

    public void setPID(double[] pid){
        this.pid = pid;
    }

    //SECTION 8 : Sensor Values

    //If 2-wheel odo, length can be set to 0.
    //Can add dependencies for different odo configurations later.
    //Order (in 3-wheel) is left, then right, then front
    public void startOdometryTracking(int width, int length, String... trackingWheelNames) {
        if(thisType != type.Odometry) return;
        odometryWheels = new DcMotorImplEx[trackingWheelNames.length];
        for(int i = 0; i < trackingWheelNames.length; i++) {
            odometryWheels[i] = hwMap.get(DcMotorImplEx.class, trackingWheelNames[i]);
        }
        double[] dimensions = new double[]{width, length};
        if(odoThread != null && odoThread.isAlive()) odoThread.Stop();
        odoThread = new OdometryThread(odometryWheels.length, this, vals, dimensions);
        odoThread.start();
    }

    public void setIMUUnit(BNO055IMU.AngleUnit Unit){
        if(thisType != type.IMU) {
            System.out.println("Invalid type.");
            return;
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        switch (Unit) {
            case DEGREES:
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                break;
            case RADIANS:
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                break;
        }
        imu.initialize(parameters);
    }
}