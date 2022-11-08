package org.firstinspires.ftc.teamcode.ours.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CupDetectorPowerPlay extends OpenCvPipeline {

    Mat HSVMat = new Mat();

    public Scalar lowerHSV = new Scalar(0, 42.5, 172.8); // the lower limit for the detection (tune this for camera)
    public Scalar upperHSV = new Scalar(120.4, 255, 255); // upper limit also tune this with the camera

    public double blurConstant = 1; // change this to change the Gaussian blur amount

    public double dilationConstant = 2; // tune

    public String cupSide = null;

    private final Object sync = new Object();

    Telemetry telemetryOpenCV = null;

    public int yellowLow = 0;
    public int yellowHigh = 0;

    public int blueLow = 0;
    public int blueHigh = 0;

    public int greenLow = 0;
    public int greenHigh = 0;

    public Rect detectROI = new Rect(0, 0 , 100, 100);

    // constructor
    public CupDetectorPowerPlay(Telemetry OpModeTelemetry) {
        telemetryOpenCV = OpModeTelemetry;
    }

    public String getCupPosition() {
        synchronized (sync) {
            return cupSide;
        }
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(HSVMat, lowerHSV, upperHSV, HSVMat);

        Size kernelSize = new Size(blurConstant, blurConstant);

        // adds blur effect to the image to help image processing
        Imgproc.GaussianBlur(HSVMat, HSVMat, kernelSize, 0);
        Size kernelRectangleSize = new Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelRectangleSize); // dialution
        Imgproc.dilate(HSVMat, HSVMat, kernel);

        synchronized (sync) {
            Mat cropped = new Mat(HSVMat, detectROI);

            MatOfDouble average = new MatOfDouble();
            MatOfDouble std = new MatOfDouble();
            Core.meanStdDev(cropped, average, std);
            double averageDouble = average.toArray()[0];

            if((averageDouble >= yellowLow) && (averageDouble <= yellowHigh)){
                cupSide = "yellow";
            }
            if((averageDouble >= blueLow) && (averageDouble <= blueHigh)){
                cupSide = "blue";
            }
            if((averageDouble >= greenLow) && (averageDouble <= greenHigh)){
                cupSide = "green";
            }
        }
       // telemetryOpenCV.update();
        return input;
    }
}