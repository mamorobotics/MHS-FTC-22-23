package org.firstinspires.ftc.teamcode.ours.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetectorPowerPlay extends OpenCvPipeline {

    Mat HSVMat = new Mat();

    // Lower and upper HSV limits for detection
    public Scalar lowerHSV = new Scalar(0, 42.5, 172.8), upperHSV = new Scalar(120.4, 255, 255);

    // 0: Yellow, 1: Blue, 2: Green
    public int copColor = -1;

    private final Object sync = new Object();

    Telemetry telemetryOpenCV;

    // Size of the images
    public int lowX = 0;
    public int lowY = 0;
    public int upX = 320;
    public int upY = 220;
    
    // Size of the cup
    public int cupWidth = 30;
    public int cupHeight = 60;
    
    // Ranges of the colors
    public double orangeLow = 30;
    public double orangeHigh = 50;

    public double magentaLow = 210;
    public double magentaHigh = 235;

    public double greenLow = 80;
    public double greenHigh = 100;

    public double blurConstant = 1;
    public double dilationConstant = 2;

    // Constructor
    public ConeDetectorPowerPlay(Telemetry OpModeTelemetry) {
        telemetryOpenCV = OpModeTelemetry;
    }

    public int getCupPosition() {
        synchronized (sync) {
            return copColor;
        }
    }

    private int getColor(int x, int y, Mat mat, int scale) { // 0: yellow, 1: magenta, 2: green
        // Obtaining the cropped mat
        Mat cropped = new Mat(mat, new Rect(x, y, scale, scale));

        // Obtaining the average of the cropped mat
        MatOfDouble average = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();
        Core.meanStdDev(cropped, average, std);
        double[] averageArray = average.toArray();
        double averageDouble = averageArray[0];
        // Determining the color of the cropped mat based on the average
        if ((averageDouble >= orangeLow) && (averageDouble <= orangeHigh)) {
            return 0;
        } else if ((averageDouble >= magentaLow) && (averageDouble <= magentaHigh)) {
            return 1;
        } else if ((averageDouble >= greenLow) && (averageDouble <= greenHigh)) {
            return 2;
        }
        return -1;
    }

    public int getCopColor() {
        return copColor;
    }

    @Override
    public Mat processFrame(Mat input) {

        copColor = -1;

        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV_FULL); //Convert to HSV

        Size kernalSize = new Size(blurConstant, blurConstant); // Gaussian Blur
        Imgproc.GaussianBlur(HSVMat, HSVMat, kernalSize, 0);
        Size kernalRectangleSize = new Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernalRectangleSize); // dialution
        Imgproc.dilate(HSVMat, HSVMat, kernal);

        // Add other required preprocessing

        int xStep = cupWidth / 3;
        int yStep = cupHeight / 3;
        int subSize = 3, buffer = 0;
        int xLength = ((int)((upX - lowX) / xStep) / subSize) * subSize;
        int yLength = ((int)((upY - lowY) / yStep) / subSize) * subSize;

        int yellow = 0, blue = 0, green = 0;

        synchronized (sync) {

            // Outer sampling
            for(int yIndex = 0; yIndex < yLength; yIndex++) {
                for(int xIndex = 0; xIndex < xLength; xIndex++) {

                    // Inner sampling
                    for (int innerYIndex = 0; innerYIndex < subSize; innerYIndex++) {
                        for (int innerXIndex = 0; innerXIndex < subSize; innerXIndex++) {
                            int x = (xIndex + innerXIndex) * xStep + lowX;
                            int y = (yIndex + innerYIndex) * yStep + lowY;

                            int color = getColor(x, y, HSVMat, 3);
                            if (color == 0) {
                                Imgproc.circle(input, new Point(x, y), 2, new Scalar(255, 245, 0), 2); //drawing circles in sample regions to visualize
                            } else if (color == 1) {
                                Imgproc.circle(input, new Point(x, y), 2, new Scalar(255, 0, 255), 2);
                            } else if (color == 2) {
                                Imgproc.circle(input, new Point(x, y), 2, new Scalar(0, 255, 0), 2);
                            } else {
                                Imgproc.circle(input, new Point(x, y), 2, new Scalar(0, 0, 0), 2);
                            }
                            Imgproc.putText(input, Integer.toString(copColor),new Point(240,120), 0, .5, new Scalar(0,0,255));
                            // Tallying the colors
                            switch (color) {
                                case 0:
                                    yellow++;
                                    break;
                                case 1:
                                    blue++;
                                    break;
                                case 2:
                                    green++;
                                    break;
                            }

                            // Making a decision if a tally has reached the threshold
                            if (yellow >= subSize * subSize - buffer) {
                                copColor = 0;
                            } else if (blue >= subSize * subSize - buffer) {
                                copColor = 1;
                            } else if (green >= subSize * subSize - buffer) {
                                copColor = 2;
                            }

                        }
                    }

                    // Decrementing each counter to prevent random pixels from skewing the decision
                    yellow = 0;
                    blue = 0;
                    green = 0;
                }
            }
        }

        // Update the telemetry with the cup color for testing
        //telemetryOpenCV.addData("Cup Color: ", copColor);
       //  telemetryOpenCV.update();
        return input;
    }
}