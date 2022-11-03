package org.firstinspires.ftc.teamcode.ours.powerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//Autonomous(name = "Close_To")
public class close_to_same_terminal_red extends LinearOpMode {
    Pose2d startPos = new Pose2d(-36, -62, Math.toRadians(270));
    int cupSide = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);

        //Sequences
        TrajectorySequence baseSeq = drive.trajectorySequenceBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(270)))
                .back(48)
                .turn(Math.toRadians(45))
                .waitSeconds(0.5)
                .turn(Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .forward(20)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                .turn(Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(new Pose2d(-48,12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(0)))
                .build();
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d(-48,12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(270)))
                .build();
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(new Pose2d(-48,12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(270)))
                .build();
        TrajectorySequence seqNone = drive.trajectorySequenceBuilder(new Pose2d(-48,12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-12, -12,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-12, -62, Math.toRadians(270)))
                .build();

        waitForStart();
        drive.followTrajectorySequence(baseSeq);
        if(cupSide == 1) {
            drive.followTrajectorySequence(seq1);
        }
        if(cupSide == 2) {
            drive.followTrajectorySequence(seq2);
        }
        if(cupSide == 3) {
            drive.followTrajectorySequence(seq3);
        }
        else{
            drive.followTrajectorySequence(seqNone);
        }
    }
}
