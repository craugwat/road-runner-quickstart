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
        coachBotBuilder.setDimensions(14,14);
        coachBotBuilder.setColorScheme(new ColorSchemeBlueDark());

        DriveShim drive = coachBotBuilder.build().getDrive();  // TODO can we pass this to a new class that adds the trajectories?
        TrajectorySequenceBuilder trajSeqBuilder = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0));
        trajh.trajSeqbuilder(trajSeqBuilder);
        RoadRunnerBotEntity coachBot = coachBotBuilder.followTrajectorySequence(trajSeqBuilder.build());

// CAW - END

// CAW - WorldClassAuto Start
//        trajHolder<TrajectorySequenceBuilder> trajh = new trajHolder();
        DefaultBotBuilder coachWorldClass1 =  new DefaultBotBuilder(meepMeep);
        coachWorldClass1.setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15);
        coachWorldClass1.setDimensions(14,14);
        coachWorldClass1.setColorScheme(new ColorSchemeBlueDark());

        DriveShim driveWC1 = coachWorldClass1.build().getDrive();  // TODO can we pass this to a new class that adds the trajectories?
        TrajectorySequenceBuilder trajSB_WC1 = driveWC1.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(180)));
        trajSB_WC1.lineTo(new Vector2d(12, -36));
        trajSB_WC1.setReversed(true);
        trajSB_WC1.waitSeconds(1);
        trajSB_WC1.splineTo(new Vector2d(56, -56), Math.toRadians(0));
        trajSB_WC1.waitSeconds(1);

        for (int i=0;i<4;i++) {
            trajSB_WC1.setReversed(false);
            trajSB_WC1.splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(180)), Math.toRadians(180)); // under bars
            trajSB_WC1.splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(180)), Math.toRadians(180)); // to piixel stack
            trajSB_WC1.waitSeconds(1);  // pick up 2 mixels
            trajSB_WC1.setReversed(true);
            trajSB_WC1.splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(180)), Math.toRadians(0)); // under bars
            trajSB_WC1.splineTo(new Vector2d(56, -56), Math.toRadians(0));
            trajSB_WC1.waitSeconds(1);
        }


        RoadRunnerBotEntity coachBot_WC1 = coachWorldClass1.followTrajectorySequence(trajSB_WC1.build());
// CAW - WorldClassAuto End



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
 //               .addEntity(coachBot)
                .addEntity(coachBot_WC1)
                .start();
    }

}