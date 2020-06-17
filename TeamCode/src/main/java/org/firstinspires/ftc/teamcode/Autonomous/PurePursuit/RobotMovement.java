package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

public class RobotMovement {

    public static void goToPosition(double x, double y, double movementSpeed){
        Point point1 = new Point(x,y);
        Point robotPosition = new Point(0,0);
        double distanceToTarget = Geometry.distanceBetweenPoints(robotPosition, point1);

        double worldAngle = 0;
        double absoluteAngleToTarget = AngleFunction.turnAngle(robotPosition, point1);

        double relativeAngleToPoint = AngleFunction.AngleWrap(absoluteAngleToTarget - (Math.toRadians(worldAngle) - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.asin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint)); //guratness movement x power to be from 0 -1
        double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint))+ Math.abs(relativeYToPoint);

        // import our actual motors functions
        //double movement_x = movementXPower;
        //double movement_y = movementYPower;
    }

}


