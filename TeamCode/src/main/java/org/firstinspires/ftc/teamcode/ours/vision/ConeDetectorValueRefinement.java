package org.firstinspires.ftc.teamcode.ours.vision;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ConeDetectorValueRefinement")
public class ConeDetectorValueRefinement extends LinearOpMode {

    int selectorState;
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
            if(gamepad1.y){
                selectorState = 0;
            }
            if(gamepad1.b){
                selectorState = 1;
            }
            if(gamepad1.a){
                selectorState = 2;
            }

            if(selectorState == 0) {
                telemetry.addData("Selector State", "Orange");
                telemetry.addData("High", (int)pipeline.orangeHigh);
                telemetry.addData("Low", (int)pipeline.orangeLow);

                pipeline.orangeHigh -= gamepad1.left_stick_y/1000;
                pipeline.orangeLow -= gamepad1.right_stick_y/1000;
            }
            if(selectorState == 1) {
                telemetry.addData("Selector State", "Magenta");
                telemetry.addData("High", (int)pipeline.magentaHigh);
                telemetry.addData("Low", (int)pipeline.magentaLow);

                pipeline.magentaHigh -= gamepad1.left_stick_y/1000;
                pipeline.magentaLow -= gamepad1.right_stick_y/1000;
            }
            if(selectorState == 2) {
                telemetry.addData("Selector State", "Green");
                telemetry.addData("High", (int)pipeline.greenHigh);
                telemetry.addData("Low", (int)pipeline.greenLow);

                pipeline.greenHigh -= gamepad1.left_stick_y/1000;
                pipeline.greenLow -= gamepad1.right_stick_y/1000;
            }

            telemetry.addData("color",pipeline.getCopColor());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("color", pipeline.copColor);
            telemetry.update();
        }
    }
}
