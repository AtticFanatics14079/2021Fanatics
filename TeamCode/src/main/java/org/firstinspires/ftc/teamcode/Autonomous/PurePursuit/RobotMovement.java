package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import java.util.ArrayList;


public class RobotMovement {
    Point robotPosition = new Point(0,0);//NEED TO UPDATE ROBOT WORLD POSIITON
    double worldAngle = 0;  //NEED TO UPDATE WORLD ANGLE
    DcMotorImplEx[] Motors = new DcMotorImplEx[4];

    //add edge cases
    public RobotMovement(HardwareMap hwMap){
        Motors[0] = hwMap.get(DcMotorImplEx.class, "back_left_motor");
        Motors[1] = hwMap.get(DcMotorImplEx.class, "front_left_motor");
        Motors[2] = hwMap.get(DcMotorImplEx.class, "front_right_motor");
        Motors[3] = hwMap.get(DcMotorImplEx.class, "back_right_motor");

        Motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        Motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        Motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        for(int i = 0; i<allPoints.size() -1; i++) {

        }
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robotPosition.x, robotPosition.y), allPoints.get(0).followDistance); //we can write function that using list of path points, figure where you are in path

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle,followMe.turnSpeed);
        //can go to op mode and run it
    }


    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
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

    public void goToPosition(double x, double y, double movementSpeed,double preferredAngle, double turnSpeed){
        Point point1 = new Point(x,y);
        double distanceToTarget = Geometry.distanceBetweenPoints(robotPosition, point1);

        double absoluteAngleToTarget = AngleFunction.turnAngle(point1, robotPosition);

        double relativeAngleToPoint = AngleFunction.AngleWrap(absoluteAngleToTarget - (Math.toRadians(worldAngle) - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint)); //guratness movement x power to be from 0 -1
        double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));

        // import our actual motors functions
        double movement_x = movementXPower;
        double movement_y = movementYPower;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movement_Turn = Range.clip(relativeTurnAngle / Math.toRadians(90), -1, 1) * turnSpeed; //need turning function and import range clip

        if(distanceToTarget < 3) {
            movement_Turn = 0;
        }
        System.out.println("Distance to target: " + distanceToTarget);
        System.out.println("MovementX: " + movement_x);
        System.out.println("MovementY: " + movement_y);
        System.out.println("MovementTurn: " + movement_Turn);
        System.out.println(relativeXToPoint);
        System.out.println(relativeYToPoint);
        setPower(-movement_x,-movement_y,0);

    }

    public void updatePose(Point pos, double worldAng){
        this.robotPosition = pos;
        this.worldAngle = worldAng;
    }

    public void setPower(double px, double py, double pa){ //Multiplied pa by -1 to suit turning requests
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        Motors[0].setPower(p1);
        Motors[1].setPower(p2);
        Motors[2].setPower(p3);
        Motors[3].setPower(p4);
    }

}


