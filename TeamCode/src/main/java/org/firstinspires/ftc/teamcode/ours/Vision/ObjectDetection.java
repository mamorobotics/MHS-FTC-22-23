package org.firstinspires.ftc.teamcode.Ours.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetection extends OpenCvPipeline {
    Scalar RED = new Scalar(255, 0, 0);


    public Scalar lowerHSV = new Scalar(19, 89, 172); // the lower limit for the detection (tune this for camera)
    public Scalar upperHSV = new Scalar(59, 250, 250); // upper limit also tune this with the camera

    private double leftBarcodeRangeBoundary = 0.3; //30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6;

    private int minRectangleArea = 500;

    //Volatile bc accessed by opmode without sync
    public volatile boolean error = false;
    public volatile Exception debug;

    private final double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    private final double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    private final double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    private double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private int loopCounter = 0;
    private int pLoopCounter = 0;

    private Mat mat = new Mat();
    private Mat processed = new Mat();
    private Mat output = new Mat();

    private Rect maxRect = new Rect(600, 1, 1, 1);

    private double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // convert to YcrCb
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV_FULL);
            Core.inRange(mat, lowerHSV, upperHSV, processed); // removes everthing that isnt in range of our color

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

            //lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);

                        // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace

                        if (rect.area() > maxArea
                                && rect.x > (borderLeftX * CAMERA_WIDTH) && rect.x + rect.width < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y > (borderTopY * CAMERA_HEIGHT) && rect.y + rect.height < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                                || loopCounter - pLoopCounter > 6) {
                            maxArea = rect.area();
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        }
                        areaPoints.release();
                    }
                    contour.release();
                }
                if (contours.isEmpty()) {
                    maxRect = new Rect();
                }
            }
            // Draw Rectangles If Area Is At Least 500
            if (first && maxRect.area() > 500) {
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
            }
            // Draw Borders
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_HEIGHT)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_WIDTH) - (borderTopY * CAMERA_HEIGHT))
            ), RED, 2);

            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }


  int getPosition(){
      if( getRectangleArea() > minRectangleArea) {
          //Then check the location of the rectangle to see which barcode it is in.
          if (getRectMidpointX() > rightBarcodeRangeBoundary * getRectWidth()) {
              return 2;
          } else if (getRectMidpointX() < leftBarcodeRangeBoundary * getRectWidth()) {
              return 0;
          }
      }
          return 1;
      }

    public double getRectangleArea(){
        synchronized (sync){
            return maxRect.area();
        }
    }

    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }

    public double getRectMidpointX() {
        synchronized (sync) {
            return maxRect.x + (maxRect.width / 2.0);
        }
    }
}