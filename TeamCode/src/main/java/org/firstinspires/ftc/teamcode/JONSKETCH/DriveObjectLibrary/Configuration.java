package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public interface Configuration {

    ArrayList<DriveObject> hardware = new ArrayList<>();
    void Configure(HardwareMap hwMap, ValueStorage vals);
    void setBulkCachingManual();
    void clearBulkCache();
}
