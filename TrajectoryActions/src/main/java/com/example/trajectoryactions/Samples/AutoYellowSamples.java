package com.example.trajectoryactions.Samples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;

public class AutoYellowSamples extends commonTrajectories {
    public AutoYellowSamples(Drive d) {
        super(d);
    }

    public SequentialAction allYellows (Drive drive) {

        Pose2d basketPos = new Pose2d(-56,-56,Math.PI/4);

        drive.setPose(transform(startPosF2));

        // *** tried to use fresh() for starting point of next trajectory but did not work.
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.getPose())
//                .splineToSplineHeading(transform(-43 ,-26, Math.PI), xformHeading(Math.PI));
//        Action toFirstYellow = tab1.build();
//        TrajectoryActionBuilder tab2 = tab1.fresh();
//        Action firstYellow2Basket = (tab2
//                .setReversed(true)
//                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4)).build());

        Action toFirstYellow = drive.actionBuilder(drive.getPose())
                .splineToSplineHeading(transform(-43 ,-26, Math.PI), xformHeading(Math.PI)).build();

        Action firstYellow2Basket = drive.actionBuilder(drive.findEndPos(toFirstYellow))
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4)).build();

//        Action toSecondYellow = drive.actionBuilder(drive.findEndPos(firstYellow2Basket))
//                .splineToSplineHeading(transform(-53 ,-26, Math.PI), xformHeading(Math.PI)).build();
//
//        Action secondYellow2Basket = drive.actionBuilder(drive.findEndPos(toSecondYellow))
//                .setReversed(true)
//                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4)).build();
//
//        Action toThirdYellow = drive.actionBuilder(drive.findEndPos(secondYellow2Basket))
//                .splineToSplineHeading(transform(-63 ,-26, Math.PI), xformHeading(Math.PI)).build();
//
//        Action thirdYellow2Basket = drive.actionBuilder(drive.findEndPos(toThirdYellow))
//                .setReversed(true)
//                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4)).build();
//
//        Action toFourthYellow = drive.actionBuilder(drive.findEndPos(thirdYellow2Basket))
//                .splineToSplineHeading(transform(-24 ,-12, 0), xformHeading(0)).build();
//
//        Action fourthYellow2Basket = drive.actionBuilder(drive.findEndPos(toFourthYellow))
//                .setReversed(true)
//                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4)).build();
//
//        Action park = drive.actionBuilder(drive.findEndPos(fourthYellow2Basket))
//                .splineToSplineHeading(transform(-24 ,-12, 0), xformHeading(0)).build();

        return new SequentialAction(
                toFirstYellow,
                actionParameters.collectSample,
//                new ParallelAction(
                        firstYellow2Basket
//                        actionParameters.liftUp
//                ),
//                actionParameters.deliverSample,
//                toSecondYellow,
//                actionParameters.collectSample,
//                secondYellow2Basket,
//                actionParameters.deliverSample,
//                toThirdYellow,
//                actionParameters.collectSample,
//                thirdYellow2Basket,
//                actionParameters.deliverSample,
//                toFourthYellow,
//                actionParameters.collectSample,
//                fourthYellow2Basket,
//                actionParameters.deliverSample,
//                park
        );
    }
}
