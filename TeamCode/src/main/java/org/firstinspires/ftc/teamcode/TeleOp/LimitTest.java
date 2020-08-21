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
        double servoPos = 0.5;
        boolean direction = true;
        boolean pressed = false;
        while(!isStopRequested()) {
            if((config.isPressed1() || config.isPressed2()) && !pressed){
                direction = !direction;
                pressed = true;
            }
            else if (!config.isPressed1() && !config.isPressed2()){
                pressed = false;
            }
            if(direction){
                servoPos += 0.01;
            }
            else{
                servoPos -= 0.01;
            }
            config.servo.setPosition(servoPos);
            telemetry.addData("Limit1: ", config.isPressed1());
            telemetry.addData("Limit2: ", config.isPressed2());
            telemetry.update();
            sleep(10);
        }
    }
}
