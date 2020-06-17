package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;
import java.util.ArrayList;


public class RobotMovement {
    static Point robotPosition = new Point(0,0);//NEED TO UPDATE ROBOT WORLD POSIITON
    static double worldAngle = 0;  //NEED TO UPDATE WORLD ANGLE

    //add edge cases
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        for(int i = 0; i<allPoints.size() -1; i++) {

        }
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robotPosition.x, robotPosition.y), allPoints.get(0).followDistance); //we can write function that using list of path points, figure where you are in path

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle,followMe.turnSpeed);
        //can go to op mode and run it
    }



    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0)); //default go to very first point

        for(int i = 0; i<pathPoints.size() - 1; i++) { //prefer points that are later in the list
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = Geometry.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
            // we want points closest to robot angle

            double closestAngle = 1000;
            closestAngle = 1000;

            for(Point thisIntersection: intersections){
                double angle = Math.atan2(thisIntersection.y - robotPosition.y, thisIntersection.x - robotPosition.x); // absolute angle to world coordinate space
                double deltaAngle = Math.abs(AngleFunction.AngleWrap(angle - Math.toRadians(worldAngle))); //his code had _rad after worldAngle

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
                // if angle is the same returns zero, otherwise corrects


            }

        }
        return followMe;


    }

    public static void goToPosition(double x, double y, double movementSpeed,double preferredAngle, double turnSpeed){
        Point point1 = new Point(x,y);
        double distanceToTarget = Geometry.distanceBetweenPoints(robotPosition, point1);

        double absoluteAngleToTarget = AngleFunction.turnAngle(robotPosition, point1);

        double relativeAngleToPoint = AngleFunction.AngleWrap(absoluteAngleToTarget - (Math.toRadians(worldAngle) - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.asin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint)); //guratness movement x power to be from 0 -1
        double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint))+ Math.abs(relativeYToPoint);

        // import our actual motors functions
        //double movement_x = movementXPower;
        //double movement_y = movementYPower;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        //movement_Turn = Range.clip(relativeTurnAngle / Math.toRadians(90), -1, 1) * turnSpeed //need turning function and import range clip

        if(distanceToTarget < 10) {
            //movementTurn = 0;
        }


    }


}


