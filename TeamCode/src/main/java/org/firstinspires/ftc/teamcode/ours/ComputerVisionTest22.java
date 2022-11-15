 package org.firstinspires.ftc.teamcode.ours;

 import android.annotation.SuppressLint;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.openftc.easyopencv.OpenCvCamera;
 import org.openftc.easyopencv.OpenCvCameraFactory;
 import org.openftc.easyopencv.OpenCvCameraRotation;

 @Autonomous(name = "visionTest22")
 public class ComputerVisionTest22 extends LinearOpMode {
     OpenCvCamera webcam; // webcam object
     CupDetectorPowerPlay detector = new CupDetectorPowerPlay(telemetry);

     @SuppressLint("DefaultLocale")
     @Override
     public void runOpMode() throws InterruptedException {
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
         webcam.setPipeline(detector);
         webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
             @Override
             public void onOpened() {
                 webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
             }

             @Override
             public void onError(int errorCode) {
                 telemetry.addData("error!!!!1",1);
             }

         });

        while(!opModeIsActive() && !isStopRequested()){
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); // This might go here to make DS viewing work
            int cupSide = detector.getCupPosition(); // gets the pos of the duck
            String cupColor;
            if(cupSide == 0){cupColor = "Yellow";}
            if(cupSide == 1){cupColor = "Blue";}
            if(cupSide == 2){cupColor = "Green";}
            else{cupColor = "No Color";}
            telemetry.addData("Cup Color: ", cupColor);
            //adds a bunch of data to benchmark pipeline and controller hub
            telemetry.update();
            sleep(100);
        }
        waitForStart();
        int snapshot = detector.getCupPosition();
        while (opModeIsActive()){
            telemetry.addData("snapshot analysis", snapshot);
        }
     }
 }
