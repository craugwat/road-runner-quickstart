package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.example.trajectoryholder.trajHolder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
//        trajHolder myTraj = new trajHolder();
        trajHolder myTraj;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .splineTo(new Vector2d(30, 36), 0)
                                .splineTo(new Vector2d(60, 0), Math.PI * 3/2)
                               .build()
                );


// CAW - expand out the above structure to see how this works.
        trajHolder<TrajectorySequenceBuilder> trajh = new trajHolder();
        DefaultBotBuilder coachBotBuilder =  new DefaultBotBuilder(meepMeep);
        coachBotBuilder.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15);
        coachBotBuilder.setDimensions(10,10);
        coachBotBuilder.setColorScheme(new ColorSchemeBlueDark());

        DriveShim drive = coachBotBuilder.build().getDrive();  // TODO can we pass this to a new class that adds the trajectories?
        TrajectorySequenceBuilder trajSeqBuilder = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0));
        trajh.trajSeqbuilder(trajSeqBuilder);
        RoadRunnerBotEntity coachBot = coachBotBuilder.followTrajectorySequence(trajSeqBuilder.build());

// CAW - END


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
                .addEntity(coachBot)
                .start();
    }

}