package org.firstinspires.ftc.teamcode.ours.powerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ours.vision.ConeDetectorPowerPlay;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Close_To_Same_Blue")
public class Close_to_same_terminal_blue extends LinearOpMode {
    Pose2d startPos = new Pose2d(36, 62, Math.toRadians(270));
    int cupSide = 3;
    OpenCvCamera webcam;
    ConeDetectorPowerPlay detector = new ConeDetectorPowerPlay(telemetry);

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

        //Sequences


        while (!opModeIsActive() && !isStopRequested()) {
            int cupColor = -1;
            if(detector.getCopColor() != -1 && cupColor == -1) {
                cupColor = detector.getCopColor();
            }

            telemetry.addData("Cup Color", cupColor);
            telemetry.update();
            sleep(100);
        }
        waitForStart();

        TrajectorySequence baseSeq = drive.trajectorySequenceBuilder(startPos)
                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(-45)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
                .forward(20)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(36, 24, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(baseSeq);
        /*TrajectorySequence seq1 = drive.trajectorySequenceBuilder(new Pose2d(48,12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(60, 36, Math.toRadians(0)))
                .build();
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d(48,12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(270)))
                .build();
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(new Pose2d(48,12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(270)))
                .build();
        TrajectorySequence seqNone = drive.trajectorySequenceBuilder(new Pose2d(48,12, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(12, 12,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(12, 62, Math.toRadians(270)))
                .build(); */
//        if(cupSide == 0) {
//            drive.followTrajectorySequence(seq1);
//        }
//        if(cupSide == 1) {
//            drive.followTrajectorySequence(seq2);
//        }
//        if(cupSide == 2) {
//            drive.followTrajectorySequence(seq3);
//        }
//        else{
//            drive.followTrajectorySequence(seqNone);
//        }
    }
}
