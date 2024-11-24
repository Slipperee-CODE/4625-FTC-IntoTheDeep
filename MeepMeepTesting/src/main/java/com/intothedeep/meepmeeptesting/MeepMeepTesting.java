package com.intothedeep.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        /* RED SIDE SPECIMEN 1+3
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -64, Math.PI/2))
                .lineToYConstantHeading(-52)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(25, -44, Math.toRadians(45)), 0)
                .waitSeconds(1)
                .turn(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(32, -44, Math.toRadians(-90)),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(9, -46, Math.toRadians(90)),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(28, -44, Math.toRadians(40)), 0)
                .waitSeconds(1)
                .turn(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(32, -44, Math.toRadians(-90)),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(9, -46, Math.toRadians(90)),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(31, -44, Math.toRadians(35)), 0)
                .waitSeconds(1)
                .turn(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(32, -44, Math.toRadians(-90)),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(9, -46, Math.toRadians(90)),0)
                .waitSeconds(1)
                .turn(Math.toRadians(-110))
                .build());
         */

        /* RED SIDE SAMPLE 1+3
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-9, -64, Math.PI/2))
                .lineToYConstantHeading(-52)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-48, -46, Math.PI/2), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, -46, Math.PI/2), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, -46, 2*Math.PI/3), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-24, 8, 0),0)
                .waitSeconds(1)
                .build());
         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}