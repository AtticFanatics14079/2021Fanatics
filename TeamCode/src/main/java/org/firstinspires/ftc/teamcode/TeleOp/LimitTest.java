package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.LimitConfiguration;

@TeleOp
public class LimitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LimitConfiguration config = new LimitConfiguration();
        config.Configure(hardwareMap);
        while(!isStopRequested()) {
            telemetry.addData("PLOX WORK: ", config.isPressed1());
            telemetry.update();
        }
    }
}
