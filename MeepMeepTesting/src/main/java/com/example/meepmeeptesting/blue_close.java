package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class blue_close {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity mybot = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36,62,Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(270)))
                                .back(48)
                                .turn(Math.toRadians(-45))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(45))
                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
                                .forward(20)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(48, 12, Math.toRadians(0)))
                                .turn(Math.toRadians(90))
                                .waitSeconds(0.5)
                                //substation
//                                .lineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(12, 62, Math.toRadians(270)))
                                //one
                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(60, 36, Math.toRadians(180)))
                                //two
//                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(270)))
                                //three
//                                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(270)))

                                .build()
                        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(mybot)
                .start();
    }
}
