package org.firstinspires.ftc.teamcode.ours.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CupDetectorPowerPlayTally extends OpenCvPipeline {

    Mat HSVMat = new Mat();

    public Scalar lowerHSV = new Scalar(0, 42.5, 172.8); // the lower limit for the detection (tune this for camera)
    public Scalar upperHSV = new Scalar(120.4, 255, 255); // upper limit also tune this with the camera

    // 0: Yellow, 1: Blue, 2: Green
    public int cupSide = -1;

    private final Object sync = new Object();

    Telemetry telemetryOpenCV;

    public int lowX = 0;
    public int lowY = 0;
    public int upX = 320;
    public int upY = 240;

    public int cupWidth = 60;
    public int cupHeight = 120;

    public int yellowLow = 55;
    public int yellowHigh = 75;

    public int blueLow = 195;
    public int blueHigh = 215;

    public int greenLow = 115;
    public int greenHigh = 145;

    // constructor
    public CupDetectorPowerPlayTally(Telemetry OpModeTelemetry) {
        telemetryOpenCV = OpModeTelemetry;
    }

    public int getCupPosition() {
        synchronized (sync) {
            return cupSide;
        }
    }

    public int getColor(int x, int y, Mat mat, int scale) {
        Mat cropped = new Mat(mat, new Rect(x, y, scale, scale));

        MatOfDouble average = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();
        Core.meanStdDev(cropped, average, std);
        double averageDouble = average.toArray()[0];

        if ((averageDouble >= yellowLow) && (averageDouble <= yellowHigh)) {
            return 0;
        } else if ((averageDouble >= blueLow) && (averageDouble <= blueHigh)) {
            return 1;
        } else if ((averageDouble >= greenLow) && (averageDouble <= greenHigh)) {
            return 2;
        }
        return -1;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(HSVMat, lowerHSV, upperHSV, HSVMat);

        int xStep = cupWidth / 3;
        int yStep = cupHeight / 3;
        int xLength = (upX - lowX) / xStep;
        int yLength = (upY - lowY) / yStep;
        int subSize = 2, buffer = 1;

        int yellow = 0, blue = 0, green = 0;

        synchronized (sync) {
            // Outer sampling
            for(int yIndex = 0; yIndex < yLength; yIndex += subSize) {
                for(int xIndex = 0; xIndex < xLength; xIndex += subSize) {

                    // Inner sampling
                    for (int innerYIndex = 0; innerYIndex < subSize; innerYIndex++) {
                        for (int innerXIndex = 0; innerXIndex < subSize; innerXIndex++) {
                            int x = (xIndex + innerXIndex) * xStep + lowX;
                            int y = (yIndex + innerYIndex) * yStep + lowY;
                            switch (getColor(x, y, HSVMat, 3)) {
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

                            if (yellow >= subSize * subSize - buffer) {
                                cupSide = 0;
                            } else if (blue >= subSize * subSize - buffer) {
                                cupSide = 1;
                            } else if (green >= subSize * subSize - buffer) {
                                cupSide = 2;
                            }

                        }
                    }

                    // Decrementing each counter to prevent random pixels from skewing the decision
                    yellow--;
                    blue--;
                    green--;
                }
            }
        }

        telemetryOpenCV.addData("Cup Color: ", cupSide);
        telemetryOpenCV.update();
        return input;
    }
}