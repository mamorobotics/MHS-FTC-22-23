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
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 13.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36,62,Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    //move slide
                                })
                                .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(-135)))
                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(56,12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, 12, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(90)))
                                //.lineToLinearHeading(new Pose2d(36, 24, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(180)))
                                // two [already there]
                                // one
                                //.lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)))
                                //three
                                //.lineToLinearHeading(new Pose2d(62, 36, Math.toRadians(90)))

                                .build()
                        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(mybot)
                .start();
    }
}
