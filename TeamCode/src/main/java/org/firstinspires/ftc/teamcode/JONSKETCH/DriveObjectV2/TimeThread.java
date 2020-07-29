package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class TimeThread extends Thread implements DOThread{

    private double seconds, value;
    private DriveObject drive;
    private boolean stop = false;

    public TimeThread(double value, double seconds, DriveObject drive){
        this.seconds = seconds;
        this.value = value;
        this.drive = drive;
    }

    //Potentially add a constructor for waiting seconds then setting

    public void run() {
        drive.set(value);
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(!stop) drive.set(0);
    }

    public void Stop(){
        stop = true;
    }
}
