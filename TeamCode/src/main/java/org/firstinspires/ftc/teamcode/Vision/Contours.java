package org.firstinspires.ftc.teamcode.Vision;

import android.hardware.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous
public class Contours extends LinearOpMode {

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        //all of our movement jazz
        while (opModeIsActive()) {
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat blurredMat = new Mat();
        Mat grayMat = new Mat();
        Mat black = new Mat();
        Mat CannyMat = new Mat();
        Mat ClosedMat = new Mat();
        Mat hierarchy = new Mat();
        Mat horizontalStructure = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
        enum Stage
        {
            RAW,
            GRAY,
            BLUR,
            CANNY,
            CLOSED,
            OUTPUT
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.RAW;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {

            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*rawMat = input;
            black = new Mat(input.size(), input.type(), Scalar.all(0));
            Imgproc.cvtColor(rawMat, grayMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(grayMat,grayMat, 0);
            Imgproc.threshold(grayMat, grayMat, 155, 255, Imgproc.THRESH_BINARY);
            Imgproc.GaussianBlur(grayMat, blurredMat, new Size(3,3),0,0);
            Imgproc.Canny(blurredMat, CannyMat, 30,90, 3, false);
            input.copyTo(black,CannyMat);
             */

            rawMat = input;
            black = new Mat(input.size(), input.type(), Scalar.all(0));
            Imgproc.cvtColor(rawMat, grayMat, Imgproc.COLOR_BGR2GRAY);
            Imgproc.GaussianBlur(grayMat, blurredMat, new Size(3,3),0,0);
            Imgproc.Canny(blurredMat, blurredMat, 50, 150, 3, false);
            input.copyTo(black, blurredMat);

            Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(21,21), new Point(10,10));
            Imgproc.morphologyEx(black,ClosedMat,3, element);
            Imgproc.cvtColor(ClosedMat,ClosedMat, Imgproc.COLOR_RGB2GRAY);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(ClosedMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Mat drawing = Mat.zeros(ClosedMat.size(), CvType.CV_8UC3);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(0, 255, 0);
                Imgproc.drawContours(drawing, contoursPolyList, i, color);
                //Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
                Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
                Imgproc.putText(drawing, "Points: " + contoursPoly[i].rows(), boundRect[i].tl(), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
            }

            /*for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(0, 255, 255);
                Imgproc.drawContours(drawing, contours, i, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            }
             */

            switch (stageToRenderToViewport) {
                case RAW: {
                    return rawMat;
                }
                case BLUR: {
                    return blurredMat;
                }
                case GRAY: {
                    return grayMat;
                }
                case CANNY: {
                    return CannyMat;
                }
                case CLOSED: {
                    return ClosedMat;
                }
                case OUTPUT: {
                    return drawing;
                }
                default:
                {
                    return input;
                }
            }
        }

    }



}
