package org.firstinspires.ftc.teamcode.ours.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CupDetectorPowerPlay extends OpenCvPipeline {
    Mat input = new Mat();

    public int cupSide = -1;

    private final Object sync = new Object();

    Telemetry telemetryOpenCV = null;

    public int topX = 220;
    public int topY = 52;
    public int bottomX = 512;

    public int cupWidth = 60;
    public int cupHeight = 120;

    public int yellowLow = 55;
    public int yellowHigh = 75;

    public int blueLow = 195;
    public int blueHigh = 215;

    public int greenLow = 115;
    public int greenHigh = 145;

    // constructor
    public CupDetectorPowerPlay(Telemetry OpModeTelemetry) {
        telemetryOpenCV = OpModeTelemetry;
    }

    public int getCupPosition() {
        synchronized (sync) {
            return cupSide;
        }
    }

    public double getColor(int x, int y, Mat img, int scale) {
        Mat hsvimg = new Mat();
        Imgproc.cvtColor(img, hsvimg, Imgproc.COLOR_RGB2HSV);
        Mat cropped = new Mat(hsvimg, new Rect(x, y, scale, scale));

        MatOfDouble average = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();
        Core.meanStdDev(cropped, average, std);
        double averageDouble = average.toArray()[0];

        if((averageDouble >= yellowLow) && (averageDouble <= yellowHigh)){
            return 0;
        }
        if((averageDouble >= blueLow) && (averageDouble <= blueHigh)){
            return 1;
        }
        if((averageDouble >= greenLow) && (averageDouble <= greenHigh)){
            return 2;
        }
        return -1;
    }

    @Override
    public Mat processFrame(Mat input) {
        int xLength = (bottomX-topX)/(cupWidth/2)+1;
        double[] colors = new double[xLength*2];

        synchronized (sync) {
            for(int yindex=1; yindex<=2; yindex++){
                for(int xindex=1; xindex<xLength; xindex++){
                    int xabsolute = xindex*(cupWidth/2)+topX;
                    int yabsolute = yindex*(cupHeight/3)+topY;
                    double color = getColor(xabsolute, yabsolute, input, 3);
                    Imgproc.circle(input, new Point(xabsolute, yabsolute), 2, new Scalar((color+1)*(255/3), 0, 0), 2);
                    colors[xindex*yindex] = getColor(xabsolute, yabsolute, input, 3);
                }
            }

            for (int i = 0; i<=xLength; i++){
                if(colors[i] == -1) {
                    cupSide = (int)colors[i];
                }
            }
        }

        Imgproc.rectangle(input, new Point(topX, topY), new Point(bottomX, topY+cupHeight), new Scalar(255, 0, 0), 2);
        telemetryOpenCV.addData("Cup Color:", cupSide);
        telemetryOpenCV.update();
        return input;
    }
}