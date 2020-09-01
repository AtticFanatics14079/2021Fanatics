package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import java.util.Locale;


@Disabled
public class LimitConfiguration {

    HardwareMap hwMap;

    public Servo servo;
    private DigitalChannel limit;
    private DigitalChannel limit2;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        limit = hwMap.get(DigitalChannel.class, "limit");
        limit2 = hwMap.get(DigitalChannel.class, "limit2");
        servo = hwMap.get(Servo.class, "servo");

        return hwMap;
    }

    public boolean isPressed1() {
        return limit.getState();
    }

    public boolean isPressed2() {
        return limit2.getState();
    }
}
