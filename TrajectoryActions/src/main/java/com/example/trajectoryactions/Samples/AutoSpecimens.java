package com.example.trajectoryactions.Samples;

// this example demonstrates how methods can be used (and re-used) to create sequential actions
// in previous years we have used a method to create the path for each of the randomized targets
// and then reused those same paths at all of our robot start positions.
// Design the trajectories to be resuable when possible saves on code tweatking and confusion.

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;

public class AutoSpecimens extends commonTrajectories {
    public AutoSpecimens(Drive d) {
        super(d);
    }

    SequentialAction deliverSpecimen(Pose2d start, int specimen){

        Action toSubmersible = drive.actionBuilder(start)
                .setReversed(true)
                .splineToLinearHeading(transform(3*specimen ,-34, Math.PI*3/2), xformHeading(Math.PI/2)).build();

        Action toSamples = drive.actionBuilder(drive.findEndPos(toSubmersible))
                .splineToSplineHeading(transform(30 ,-36, 0), xformHeading(0))
                .splineToLinearHeading(transform(40+10*specimen ,-26, 0), xformHeading(0)).build();

        Action toObservationZone1 = drive.actionBuilder(drive.findEndPos(toSamples))
                .setReversed(true)
                .splineToLinearHeading(transform(48,-52, Math.PI*3/2), xformHeading(0)).build();

        Action toObservationZone2 = drive.actionBuilder(drive.findEndPos(toObservationZone1))
                .splineToLinearHeading(transform(48,-48, Math.PI*3/2), xformHeading(Math.PI*3/2))
                .waitSeconds(2)
                .splineToLinearHeading(transform(48,-52, Math.PI*3/2), xformHeading(Math.PI*1/2)).build();

        return new SequentialAction(
                toSubmersible,
                actionParameters.deliverSpecimen,
                toSamples,
                actionParameters.collectSample,
                toObservationZone1,
                actionParameters.deliverSample,
                toObservationZone2,
                actionParameters.CollectSpecimen
        );
    }


    public SequentialAction allSpecimens(Drive drive) {

        drive.setPose(transform(startPosF4));

        Action d1 = deliverSpecimen(drive.getPose(), 0);
        Action d2 = deliverSpecimen(drive.findEndPos(d1), 1);
        Action d3 = deliverSpecimen(drive.findEndPos(d2), 2);

        Action park = drive.actionBuilder(drive.findEndPos(d3))
                .setReversed(true)
                .splineToSplineHeading(transform(24 ,-12, Math.PI*3/2), xformHeading(Math.PI)).build();

        return new SequentialAction(
                d1,     // deliver preload, pick up 1st sample and take to  human to make specimen
                d2,     // deliver 2nd specimena dn pick up sample and have human make specimen
                park    // no time to deliver, go park.
        );
    }

}
