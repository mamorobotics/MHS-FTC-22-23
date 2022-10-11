package org.firstinspires.ftc.teamcode.Ours;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Ours.Vision.CupDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Auto extends LinearOpMode {
    OpenCvCamera webcam; // webcam object
    CupDetector detector = new CupDetector(telemetry); // duck pos object

    DcMotor FL = null;
    DcMotor FR  = null;
    DcMotor BL = null;
    DcMotor BR = null;


    //static final double wheelDiameter = 3.78;

    //static final double TICKS_PER_INCH  = TICK_PER_REV / (wheelDiameter * Math.PI);
      static final double TICKS_PER_INCH = 43.5; //43.5
    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FrontL");
        FR = hardwareMap.get(DcMotor.class, "FrontR");
        BL = hardwareMap.get(DcMotor.class, "BackL");
        BR = hardwareMap.get(DcMotor.class, "BackR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //creating the webcam stuff needed for computer vision
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

        while (!opModeIsActive()){ // the innit stage
            telemetry.addData("Duck Position",detector.getCupPosition());
            telemetry.update();
        }
    waitForStart();
        encoderDrive(1, 50,50);
        sleep(5000);
        encoderDrive(.2,-50,-50);
        sleep(500);
        encoderDrive(.4,20,-20); //left turn ?
        sleep(500);
        encoderDrive(.4,-20,20); // right turn ?
    }

    public void encoderDrive(double speed, double leftInches, double rightInches){
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        //calculates target amount of ticks to travel over the requested distance
        leftFrontTarget  = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_INCH);
        rightFrontTarget  = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_INCH);
        leftBackTarget  = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_INCH);
        rightBackTarget  = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_INCH);

        //sets the requested amount of ticks
        FR.setTargetPosition(rightFrontTarget);
        BR.setTargetPosition(rightBackTarget);
        BL.setTargetPosition(leftBackTarget);
        FL.setTargetPosition(leftFrontTarget);

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
        FL.setPower(speed);

        //this loop is needed or else it won't work
        while (BL.isBusy() || FR.isBusy() || BR.isBusy() || FL.isBusy()){
            telemetry.addData("Left Front:", FL.getCurrentPosition());
            telemetry.addData("Left Back:", BL.getCurrentPosition());
            telemetry.addData("Right Front:", FR.getCurrentPosition());
            telemetry.addData("Right Back:", BR.getCurrentPosition());
            telemetry.update();
        }

        //test to see if this is necesairy after changing the zero power behavior to break
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}