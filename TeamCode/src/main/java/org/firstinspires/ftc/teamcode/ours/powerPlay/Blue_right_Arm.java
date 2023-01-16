/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.ours.powerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ours.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@Autonomous
public class Blue_right_Arm extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static Servo arm1, arm2, clawControlServo, clawServo;
    Pose2d startPos = new Pose2d(-36, 62, Math.toRadians(270));


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode()
    {



        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawControlServo = hardwareMap.get(Servo.class, "clawControlServo");

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);

        //  arm1.setPosition(0);
        // arm2.setPosition(0);
        clawServo.setPosition(.23);
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        double starttime = this.getRuntime();
        arm1.setPosition(0);
        arm2.setPosition(0);
        clawControlServo.setPosition(0);
        while ( this.getRuntime() - starttime < 4)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful get bent */
        if(tagOfInterest == null){
            clawServo.setPosition(.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                    .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                    .turn(Math.toRadians(45))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq1);
            clawServo.setPosition(0);
            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(-135))
                    .lineToLinearHeading(new Pose2d(-39.7, 9, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(seq2);
            arm1.setPosition(0.65);
            arm2.setPosition(0.55);
            clawControlServo.setPosition(0.34);
            sleep(4000);
            clawServo.setPosition(0.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.73);
            sleep(4000);
            TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                    .turn(Math.toRadians(135))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq3);
            clawServo.setPosition(0);
            TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(45))
                    .lineToSplineHeading((new Pose2d(-36,36,Math.toRadians(0))))
                    .lineToLinearHeading(new Pose2d(-61, 36, Math.toRadians(0)))
                    .build();
            drive.followTrajectorySequence(seq4);
        }
        else if(tagOfInterest.id == RIGHT){
            clawServo.setPosition(.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                    .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                    .turn(Math.toRadians(45))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq1);
            clawServo.setPosition(0);
            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(-135))
                    .lineToLinearHeading(new Pose2d(-39.7, 9, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(seq2);
            arm1.setPosition(0.65);
            arm2.setPosition(0.55);
            clawControlServo.setPosition(0.34);
            sleep(4000);
            clawServo.setPosition(0.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            sleep(4000);
            TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                    .turn(Math.toRadians(135))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq3);
            clawServo.setPosition(0);
            TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(45))
                    .lineToSplineHeading((new Pose2d(-36,36,Math.toRadians(0))))
                    .lineToLinearHeading(new Pose2d(-61, 36, Math.toRadians(0)))
                    .build();
            drive.followTrajectorySequence(seq4);
        }
        else if(tagOfInterest.id == MIDDLE){
            clawServo.setPosition(.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                    .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                    .turn(Math.toRadians(45))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq1);
            clawServo.setPosition(0);
            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(-135))
                    .lineToLinearHeading(new Pose2d(-39.7, 9, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(seq2);
            arm1.setPosition(0.65);
            arm2.setPosition(0.55);
            clawControlServo.setPosition(0.34);
            sleep(4000);
            clawServo.setPosition(0.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            sleep(4000);
            TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                    .turn(Math.toRadians(135))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq3);
            clawServo.setPosition(0);
            TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(45))
                    .lineToSplineHeading((new Pose2d(-36,36,Math.toRadians(0))))
                    .build();
            drive.followTrajectorySequence(seq4);
        }
        else{
            clawServo.setPosition(.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                    .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                    .turn(Math.toRadians(45))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq1);
            clawServo.setPosition(0);
            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(-135))
                    .lineToLinearHeading(new Pose2d(-39.7, 9, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(seq2);
            arm1.setPosition(0.65);
            arm2.setPosition(0.55);
            clawControlServo.setPosition(0.34);
            sleep(4000);
            clawServo.setPosition(0.24);
            arm1.setPosition(0);
            arm2.setPosition(0.3);
            clawControlServo.setPosition(0.7);
            sleep(4000);
            TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                    .turn(Math.toRadians(135))
                    .lineToLinearHeading(new Pose2d(-30.5, 8, Math.toRadians(315)))
                    .waitSeconds(0.5)
                    .build();
            drive.followTrajectorySequence(seq3);
            clawServo.setPosition(0);
            TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(315)))
                    .turn(Math.toRadians(45))
                    .lineToSplineHeading((new Pose2d(-36,36,Math.toRadians(0))))
                    .lineToSplineHeading((new Pose2d(-12, 36, Math.toRadians(0))))
                    .build();
            drive.followTrajectorySequence(seq4);
        }
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
