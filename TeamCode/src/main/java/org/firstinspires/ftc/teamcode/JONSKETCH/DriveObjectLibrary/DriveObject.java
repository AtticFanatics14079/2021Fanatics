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

    private BNO055IMU imu;
    private TouchSensor touch;

    private int partNum;
    private Double[] pid = {30.0, 0.0, 0.0, 3000.0}; //Default values
    private PositionThread posThread = new PositionThread();
    private TimeThread timeThread = new TimeThread();
    private static ValueStorage vals;

    private String objectName;

    public enum type{
        DcMotorImplEx, Servo, CRServo, IMU, TouchSensor, Odometry, Null
    }

    public enum classification{
        Drivetrain, toPosition, Default, Sensor
    }

    private type thisType;
    private classification thisClass;

    public DriveObject(type typeName, String objectName, classification classifier, ValueStorage vals, int partNum, HardwareMap hwMap){

        this.objectName = objectName;

        this.vals = vals;

        this.partNum = partNum;

        thisType = typeName;

        thisClass = classifier;

        switch(thisType){
            case DcMotorImplEx:
                motor = hwMap.get(DcMotorImplEx.class, objectName);
                break;
            case Servo:
                thisClass = classification.toPosition;
                servo = hwMap.get(Servo.class, objectName);
                break;
            case CRServo:
                crservo = hwMap.get(CRServo.class, objectName);
                break;
            case IMU:
                thisClass = classification.Sensor;
                imu = hwMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
                //Add configuration of IMU here
                break;
            case TouchSensor:
                touch = hwMap.get(TouchSensor.class, objectName);
                break;
            case Odometry:
                motor = hwMap.get(DcMotorImplEx.class, objectName);
                break;
        }
    }

    public type getType(){
        return thisType;
    }

    public String getName(){
        return objectName;
    }

    public classification getClassification(){
        return thisClass;
    }

    public int getPartNum(){return partNum;}

    public void setType(type t){
        thisType = t;
    }

    public void setName(String n){
        objectName = n;
    }

    public void setClassification(classification c){
        thisClass = c;
    }

    public void setPartNum(int p){partNum = p;}

    public void setHardware(double Value){
        switch(thisType){
            case DcMotorImplEx:
                switch(thisClass){
                    case Drivetrain:
                        motor.setVelocity(Value);
                        break;
                    case toPosition:
                        motor.setVelocity(Value);
                        break;
                    case Default:
                        motor.setVelocity(Value);
                        break;
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        break;
                }
                break;
            case Servo:
                servo.setPosition(Value);
                break;
            case CRServo:
                switch(thisClass){
                    case Drivetrain:
                        crservo.setPower(Value);
                        break;
                    case Default:
                        crservo.setPower(Value);
                        break;
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        break;
                }
                break;
            default:
                System.out.println("Invalid type for setting.");
                break;
        }
    }

    public Thread set(double Value){
        Double[] p = new Double[partNum + 1];
        Boolean[] b = new Boolean[partNum + 1];
        for(int i = 0; i <= partNum; i++) {
            p[i] = null;
            b[i] = null;
        }
        switch(thisType){
            case DcMotorImplEx:
                switch(thisClass){
                    case Drivetrain:
                        p[partNum] = Value;
                        b[partNum] = true;
                        vals.changedParts(true, b);
                        vals.runValues(true, p);
                        //Maybe apply something to limit acceleration, but for now, just set power.
                        break;
                    case toPosition:
                        setTargetPosition((int) Value, 1.0);
                    case Default:
                        p[partNum] = Value;
                        b[partNum] = true;
                        vals.changedParts(true, b);
                        vals.runValues(true, p);
                        break;
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        break;
                }
                break;
            case Servo:
                p[partNum] = Value;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                break;
            case CRServo:
                switch(thisClass){
                    case Drivetrain:
                        p[partNum] = Value;
                        b[partNum] = true;
                        vals.changedParts(true, b);
                        vals.runValues(true, p);
                        //See above for further changes.
                        break;
                    case Default:
                        p[partNum] = Value;
                        b[partNum] = true;
                        vals.changedParts(true, b);
                        vals.runValues(true, p);
                        break;
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        break;
                }
                break;
            default:
                System.out.println("Invalid type for setting.");
                break;
        }
        return null;
    }

    public Double[] getHardware(){
        switch(thisType){
            case DcMotorImplEx:
                switch(thisClass){
                    case toPosition:
                        return new Double[]{(double) motor.getCurrentPosition(), motor.getVelocity()};
                    case Drivetrain:
                        return new Double[]{motor.getVelocity(), (double) motor.getCurrentPosition()};
                    case Default:
                        return new Double[]{motor.getVelocity(), (double) motor.getCurrentPosition()};
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        return null;
                }
            case Servo:
                return new Double[]{servo.getPosition(), null}; //Fill in null for all values past usable and before reaching maxValues in ValueStorage
            case CRServo:
                return new Double[]{crservo.getPower(), null};
            case IMU:
                return new Double[]{(double) imu.getAngularOrientation().firstAngle, (double) imu.getAngularOrientation().secondAngle}; //Can add more returns here
            case TouchSensor:
                return new Double[]{touch.getValue(), null};
            case Odometry:
                return new Double[]{(double) motor.getCurrentPosition(), null}; //May be able to return velocity here, not sure if rolling works for velocity.
            default:
                System.out.println("Invalid type, returning null.");
                return null;
        }
    }

    public Double get(){
        return vals.hardware(false, null)[partNum][0];
        /*switch(thisType){
            case DcMotorImplEx:
                switch(thisClass){
                    case toPosition:
                        return vals.hardware(false, null)[partNum].first;
                    case Drivetrain:
                        return vals.hardware(false, null)[partNum].first;
                    case Default:
                        return vals.hardware(false, null)[partNum].first;
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        return null;
                }
            case Servo:
                return vals.hardware(false, null)[partNum].first;
            case CRServo:
                return vals.hardware(false, null)[partNum].first;
            case IMU:
                return vals.hardware(false, null)[partNum].first;
            case TouchSensor:
                return vals.hardware(false, null)[partNum].first;
            case Odometry:
                return vals.hardware(false, null)[partNum].first;
            default:
                System.out.println("Invalid type, returning null.");
                return null;
        }

         */
    }

    public Double get(int VariableNum){
        return vals.hardware(false, null)[partNum][VariableNum];
    }

    public Double[] getAllVals(){
        return vals.hardware(false, null)[partNum];
    }

    /*public Pair<Double, Double> getAllVals(){
        switch(thisType){
            case DcMotorImplEx:
                switch(thisClass){
                    case toPosition:
                        return vals.hardware(false, null)[partNum];
                    case Drivetrain:
                        return vals.hardware(false, null)[partNum];
                    case Default:
                        return vals.hardware(false, null)[partNum];
                    default:
                        System.out.println("Invalid type/classifier combination.");
                        return null;
                }
            case Servo:
                return vals.hardware(false, null)[partNum];
            case CRServo:
                return vals.hardware(false, null)[partNum];
            case IMU:
                return vals.hardware(false, null)[partNum];
            case TouchSensor:
                return vals.hardware(false, null)[partNum];
            case Odometry:
                return vals.hardware(false, null)[partNum];
            default:
                System.out.println("Invalid type, returning null.");
                return null;
        }
    }

     */

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

    public void resetEncoders(){
        if(thisType == type.DcMotorImplEx) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setPower(double Power){
        Double[] p = new Double[partNum + 1];
        Boolean[] b = new Boolean[partNum + 1];
        for(int i = 0; i <= partNum; i++) {
            p[i] = null;
            b[i] = null;
        }
        p[partNum] = Power;
        b[partNum] = true;
        vals.changedParts(true, b);
        vals.runValues(true, p);
    }

    public Thread setPower(double Power, double Seconds){
        /*if(!(thisClass == classification.Default || thisClass == classification.Drivetrain)) {
            System.out.println("Invalid type for setting power.");
            return null;
        }

         */
        timeThread = new TimeThread(Power, Seconds, this);
        timeThread.start();
        System.out.println(timeThread.isAlive());
        return timeThread;
    }

    public Thread setTargetPosition(double targetPosition, double maxSpeed){
        if(thisClass != classification.toPosition){
            System.out.println("Invalid call.");
            return null;
        }
        switch(thisType){
            case Servo:
                Double[] p = new Double[partNum + 1];
                Boolean[] b = new Boolean[partNum + 1];
                for(int i = 0; i < partNum; i++) {
                    p[i] = null;
                    b[i] = null;
                }
                p[partNum] = targetPosition;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                return null;
            case DcMotorImplEx:
                //if(pos.isAlive()) pos.Stop(); //Currently starting a new thread breaks a part
                if(posThread.isAlive()) posThread.Stop();
                posThread = new PositionThread((int) targetPosition, maxSpeed, 50, this);
                posThread.start();
                return posThread;
        }
        return null;
    }

    public Thread setTargetPosition(double targetPosition, double maxSpeed, double tolerance){
        if(thisClass != classification.toPosition){
            System.out.println("Invalid call.");
            return null;
        }
        switch(thisType){
            case Servo:
                Double[] p = new Double[partNum + 1];
                Boolean[] b = new Boolean[partNum + 1];
                for(int i = 0; i < partNum; i++) {
                    p[i] = null;
                    b[i] = null;
                }
                p[partNum] = targetPosition;
                b[partNum] = true;
                vals.changedParts(true, b);
                vals.runValues(true, p);
                return null;
            case DcMotorImplEx:
                //if(pos.isAlive()) pos.Stop(); //Currently starting a new thread breaks a part
                if(posThread.isAlive()) posThread.Stop();
                posThread = new PositionThread((int) targetPosition, maxSpeed, tolerance, this);
                posThread.start();
                return posThread;
        }
        return null;
    }

    public Thread groupSetTargetPosition(int targetPos, double maxSpeed, double tolerance, DriveObject ...drive){
        for(DriveObject d : drive) if(d.getClassification() != classification.toPosition) return null;
        //if(pos.isAlive()) pos.Stop(); //Currently starting a new thread breaks a part
        if(posThread.isAlive()) posThread.Stop();
        posThread = new PositionThread(targetPos, maxSpeed, tolerance, drive);
        posThread.start();
        return posThread;
    }

    public Thread groupSetTargetPosition(int targetPos, double maxSpeed, double tolerance, ArrayList<DriveObject> drive){
        for(DriveObject d : drive) if(d.getClassification() != classification.toPosition) return null;
        //if(pos.isAlive()) pos.Stop(); //Currently starting a new thread breaks a part
        if(posThread.isAlive()) posThread.Stop();
        DriveObject[] d = new DriveObject[drive.size()];
        int i = 0;
        for(DriveObject a : drive) d[i++] = a;
        posThread = new PositionThread(targetPos, maxSpeed, tolerance, d);
        posThread.start();
        return posThread;
    }

    public Double[] getPID(){
        return pid;
    }

    public void setPID(double p, double i, double d){
        pid[0] = p;
        pid[1] = i;
        pid[2] = d;
    }
    public void setPIDF(double p, double i, double d, double f){
        pid[0] = p;
        pid[1] = i;
        pid[2] = d;
        pid[3] = f;
    }

    public void setPID(Double[] pid){
        this.pid = pid;
    }

    public void setUnitType(BNO055IMU.AngleUnit Unit){
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

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if(thisType == type.DcMotorImplEx) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void endAllThreads(){
        if(posThread.isAlive()) posThread.Stop();
        //if(timeThread.isAlive()) timeThread.Stop();
    }
}