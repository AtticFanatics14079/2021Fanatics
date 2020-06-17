package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

public class Geometry {
    public static double distanceBetweenPoints(Point point1, Point point2) {

        double hypotenuse = Math.hypot(point1.getx()-point2.getx(), point1.gety()-point2.gety());
        return hypotenuse;
    }

    public static void spacePoints(Point startPoint,Point endPoint, double Spacing){
        //Spacing - injecting points - desired distance between any 2 points(~6") + Array to put points in
        //Vector = end point - start point
        // number of points that fir = vector.magnitude(length) /spacing
        //vector = vector.normalize(turns vector legnth into one unit) *spacing
        //normalizing a vector: find length(pythagorean), divide each x and y coordinate by length
        // multiply it by each spacing, making it evenly spaced among the segment
        Point vector = startPoint.subtract(endPoint);
        Point[] pointsArray;
        double pointsThatFit = Math.ceil(distanceBetweenPoints(startPoint, endPoint)/Spacing);
        pointsArray = new Point[(int)pointsThatFit+1];
        Point vector_normalized = vector.normalize(vector, distanceBetweenPoints(startPoint, endPoint));
        vector_normalized = vector_normalized.multiply(Spacing);
        for(int i = 0; i< pointsThatFit ; i++){
            pointsArray[i] = startPoint.add(vector_normalized.multiply(i));
        }
        pointsArray[(int) pointsThatFit] = endPoint;

    }

}
