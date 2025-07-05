package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;




public class MeepMeepTesting {
    static boolean Imri = true;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

    if (Imri){
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -70, Math.toRadians(90)))
                .waitSeconds(1) //to see start
                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(45)) //to basket
                .waitSeconds(1) //put preload
                .lineToXSplineHeading(-48,Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(45))
                .waitSeconds(1)
                .lineToXSplineHeading(-52,Math.toRadians(90))
                            .waitSeconds(1)
//                .strafeToLinearHeading()
                .build());
    }
    else{
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -70, Math.toRadians(90)))
                .waitSeconds(2) //to see start
                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(45)) //to basket
                .waitSeconds(5) //put preload
                .lineToXSplineHeading(-48,Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(60))
                .waitSeconds(2)
                .build());
    }

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(1)
                .addEntity(myBot)
                .start();
    }
}