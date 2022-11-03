package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class milesTest_power_play {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity mybot = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34.5,59,Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(12, 59, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12, 46, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(270)))
                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(34.5, 36, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)))
                                .build()
                        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(mybot)
                .start();
    }
}
