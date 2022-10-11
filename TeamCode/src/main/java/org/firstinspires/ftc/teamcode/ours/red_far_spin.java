package org.firstinspires.ftc.teamcode.Ours;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Ours.Vision.CupDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "red_spin_far")
public class red_far_spin extends LinearOpMode {
    Pose2d startPos = new Pose2d(12, -62, Math.toRadians(90));
    static DcMotor spinner;
    static Servo bucketServo;
    static DcMotor trackMotor;
    OpenCvCamera webcam; // webcam object
    CupDetector detector = new CupDetector(telemetry);
    static int cupPos = 0;

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

                telemetry.addData("failed to open camera setting position to ", detector);
            }

        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);
        spinner = hardwareMap.get(DcMotor.class, "SpinnerMotor");
        trackMotor = hardwareMap.get(DcMotor.class, "TrackMotor");
        bucketServo = hardwareMap.servo.get("BucketServo");
        trackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setOuttakePos(telemetry, -10, .70);
        while (!opModeIsActive() && !isStopRequested()) {
            //bucketServo.setPosition(upright);
            cupPos = detector.getCupPosition(); // gets the pos of the duck
            telemetry.addData("duck pos", cupPos);
            telemetry.update();
            sleep(100);
        }
        waitForStart();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPos)
                .forward(3)
                .lineToSplineHeading(new Pose2d(2,-37,Math.toRadians(315)))
                .build();
        drive.followTrajectorySequence(trajSeq1);

        if (cupPos == 0) {
            setOuttakePos(telemetry, -950, .70);
        }
        if (cupPos == 1) {
            setOuttakePos(telemetry, -1400, .70);
        }
        if(cupPos == 2) {
            setOuttakePos(telemetry, -2050, .70);
        }
        bucketServo.setPosition(.90);
        sleep(2000);
        bucketServo.setPosition(.5);

        setOuttakePos(telemetry, -855, .70);

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(10)
                .lineToLinearHeading(new Pose2d(-57,-55,Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(trajSeq2);

        spinner.setPower(.65);
        sleep(3000);
        spinner.setPower(0);

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(10)
                .lineToLinearHeading(new Pose2d(-60,-35,Math.toRadians(0)))
                .build();

        drive.followTrajectorySequence(trajSeq3);
    }

    private void setOuttakePos(Telemetry telemetry, int position, double speed){
        trackMotor.setTargetPosition(position); //get this for the lower position tick amount

        trackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        trackMotor.setPower(speed); // adjust this low for testing

        while (trackMotor.isBusy()){
            telemetry.addData("current pos", trackMotor.getCurrentPosition());
            telemetry.update();
            if(trackMotor.getCurrentPosition() > -800) {
                bucketServo.setPosition(.11);
            } else{
                bucketServo.setPosition(.5);
            }
        }
        trackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trackMotor.setPower(0);
    }
}