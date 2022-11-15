package org.firstinspires.ftc.teamcode.ours.Vision;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TestPipeline  extends LinearOpMode {

    OpenCvCamera cam;

    ConeDetectorPowerPlay pipeline = new ConeDetectorPowerPlay(telemetry);
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cam.setPipeline(pipeline);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); // sets the resolution
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam failed to initiate",0);
            }
        });

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("color",pipeline.getCopColor());
        }
        waitForStart();
    }
}
