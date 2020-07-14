package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {

    ValueStorage vals = new ValueStorage();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleConfiguration config = new SampleConfiguration();
        HardwareThread hardware = new HardwareThread(vals, hardwareMap, config);
        hardware.start();
        for(DriveObject d : hardware.config.hardware) {
            d.setPID(2, 0.000005, 0.5);
        }
        //System.out.println(hardware.config.hardware.get(0).getPID()[0] + " hf");
        hardware.startTime(new ElapsedTime());
        Sequence s = new Sequence(() -> hardware.config.hardware.get(0).groupSetTargetPosition(3000, 1, 10, hardware.config.hardware), null);
        Sequence s2 = new Sequence(() -> hardware.config.hardware.get(0).groupSetTargetPosition(0, 1, 10, hardware.config.hardware), s);
        Thread t = new Thread(s2);
        /*Sequence t1 = new Sequence(() -> {
            hardware.config.hardware.get(1).setPower(-100, 3);
            Thread a = hardware.config.hardware.get(0).setPower(100, 2);
            System.out.println(a.isAlive());
            return a;
        }, null);
        Sequence t2 = new Sequence(() -> hardware.config.hardware.get(2).setPower(100, 2), t1);
        Thread timeThread = new Thread(t2);

         */
        waitForStart();
        //timeThread.start();
        t.start();
        while (!isStopRequested()){
            //telemetry.addData("M1 Position", hardware.config.hardware.get(0).get());
            //telemetry.addData("T1 Alive", timeThread.isAlive());
            //telemetry.update();
        }
        hardware.Stop();
    }
}
