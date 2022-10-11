package org.firstinspires.ftc.teamcode.Ours.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CupDetector extends OpenCvPipeline {

    Mat HSVMat = new Mat();
    Mat contoursOnFrameMat = new Mat();

    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;

    public Scalar lowerHSV = new Scalar(0, 42.5, 172.8); // the lower limit for the detection (tune this for camera)
    public Scalar upperHSV = new Scalar(120.4, 255, 255); // upper limit also tune this with the camera

    public double yAxisTop = 40; // the upper threshold so anything above this imaginary line is not being seen
    // by the pipeline

    public double yAxisBot = 70;

    public double blurConstant = 1; // change this to change the Gaussian blur amount

    public double maxArea = 2000;

    public double minArea = 500;

    public double dilationConstant = 2; // tune

    int duckPosition = 0; // far left = 0, middle 1 , far right 2

    private final Object sync = new Object();

    Telemetry telemetryOpenCV = null;

    // constructor
    public CupDetector(Telemetry OpModeTelemetry) {
        telemetryOpenCV = OpModeTelemetry;
    }

    public int getCupPosition() {
        synchronized (sync) {
            return duckPosition;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // clear the list
        contoursList.clear();

        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV_FULL); // converts the RGB image (Mat) to the HSV color
        // space and outputs that to HSV Mat
        Core.inRange(HSVMat, lowerHSV, upperHSV, HSVMat); // filters every pixel that is not in our range

        Size kernalSize = new Size(blurConstant, blurConstant);

        // adds blur effect to the image to help image processing
        Imgproc.GaussianBlur(HSVMat, HSVMat, kernalSize, 0);
        Size kernalRectangleSize = new Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);

        Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernalRectangleSize); // dialution
        Imgproc.dilate(HSVMat, HSVMat, kernal);
        // finds contours
        Imgproc.findContours(HSVMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        numContoursFound = contoursList.size(); // finds the amount of contours made

        input.copyTo(contoursOnFrameMat);
        synchronized (sync) {
            for (MatOfPoint contour : contoursList) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > 1000) {
                    Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                    Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.x), rect.tl(), 0, 0.5,
                            new Scalar(2500, 255, 255)); // prints x value
                    // x value for left
                    if (rect.x <= 130 && numContoursFound > 0) {
                        duckPosition = 1;
                        telemetryOpenCV.addLine("cup is in the middle");
                        // telemetryOpenCV.update();
                    } else if (rect.x >= 180 && numContoursFound > 0) {
                        duckPosition = 2;
                        telemetryOpenCV.addLine("cup is on the right");
                        // telemetryOpenCV.update();
                    }

                }

            }
            if (numContoursFound == 0) {
                duckPosition = 0;
                telemetryOpenCV.addLine("cup is on the left ");
            }
        }
       // telemetryOpenCV.update();
        return contoursOnFrameMat;
    }
}