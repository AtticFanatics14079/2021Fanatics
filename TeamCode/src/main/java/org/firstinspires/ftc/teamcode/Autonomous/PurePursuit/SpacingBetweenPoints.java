package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

public class SpacingBetweenPoints {
    //Spacing - injecting points - desired distance between any 2 points(~6") + Array to put points in
    //Vector = end point - start point
        // number of points that fir = vector.magnitude(length) /spacing
        //vector = vector.normalize(turns vector legnth into one unit) *spacing
        //normalizing a vector: find length(pythagorean), divide each x and y coordinate by length
            // multiply it by each spacing, making it evenly spaced among the segment

    public static void spacePoints(double startPoint,double endPoint, double Spacing){ //2 dimensional arrays?
        double[] pointsArray;
        Spacing = 6;
        double vector = endPoint - startPoint;
        int pointsThatFit = Math.ceil(vector.magnitude()/Spacing); //gotta get magnitude function
        pointsArray = new double[pointsThatFit];
        vector = vector.normalize() * Spacing; //gotta get normalizing function
        for(i = 0; i< pointsThatFit ; i++){
            pointsArray[i] = startPoint + vector*i;
        }
        pointsArray[pointsThatFit] = endPoint;

    }
    public static void magnitude(){

    }
    public static void normalize(){

    }
}
