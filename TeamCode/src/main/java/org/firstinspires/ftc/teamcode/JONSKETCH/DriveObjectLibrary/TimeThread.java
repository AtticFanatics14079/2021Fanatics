package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

public class TimeThread extends Thread{

    private double seconds, value, endVal;
    private DriveObject drive;

    public TimeThread(double value, double Seconds, DriveObject drive){
        seconds = Seconds;
        this.value = value;
        this.drive = drive;
        endVal = 0;
    }

    public TimeThread(double value, double Seconds, double endVal, DriveObject drive){
        seconds = Seconds;
        this.value = value;
        this.drive = drive;
        this.endVal = endVal;
    }

    public TimeThread(){}

    public void run() {
        switch(drive.getType()){
            case DcMotorImplEx:
                drive.setPower(value);
                break;
            case CRServo:
                drive.setPower(value);
                break;
            case Servo:
                drive.set(value);
            default:
                System.out.println("Invalid type for setting power.");
                return;
        }
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.setPower(value);
    }
}
