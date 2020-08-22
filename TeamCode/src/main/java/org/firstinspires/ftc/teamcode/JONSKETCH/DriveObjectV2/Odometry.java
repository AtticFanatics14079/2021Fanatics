package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import java.util.ArrayList;

public interface Odometry {

    ArrayList<DMotor> odoModules = new ArrayList<>();

    Point getPosition();
    void setPosition(Point newPos);
    double getX();
    double getY();
    double getHeading();
    int getPartNum();
    void beginTracking();
}
