 package org.firstinspires.ftc.teamcode.Ours;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Ours.Vision.CupDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "visionTest")
public class ComputerVisionTest extends LinearOpMode {
    OpenCvCamera webcam; // webcam object
    CupDetector detector = new CupDetector(telemetry);

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
                telemetry.addData("errprr!!!!1",1);
            }

        });

   while(!opModeIsActive() && !isStopRequested()){
            int pos = detector.getCupPosition(); // gets the pos of the duck
            telemetry.addData("duck pos", pos);
            //adds a bunch of data to benchmark pipeline and controller hub
            telemetry.update();
            sleep(100);        }
        waitForStart();
        int snapshot = detector.getCupPosition();
        while (opModeIsActive()){
            telemetry.addData("snapshot analysis", snapshot);
        }
    }
}
