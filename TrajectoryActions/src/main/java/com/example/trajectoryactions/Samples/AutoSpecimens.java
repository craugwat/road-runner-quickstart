package com.example.trajectoryactions.Samples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.example.trajectoryactions.SimConfig.Drive;

public class AutoSpecimens extends commonTrajectories {
    public AutoSpecimens(Drive d) {
        super(d);
    }

    int specimen = -1;
    SequentialAction deliverSpecimen(Pose2d start){
        specimen++;

        Action toSubmersible = drive.actionBuilder(start)
                .setReversed(true)
                .splineToLinearHeading(transform(3*specimen ,-34, Math.PI*3/2), xformHeading(Math.PI/2)).build();

        Action toSamples = drive.actionBuilder(drive.findEndPos(toSubmersible))
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

        int dir=1;

        drive.setPose(transform(startPosF4));

        Action d1 = deliverSpecimen(drive.getPose());
        Action d2 = deliverSpecimen(drive.findEndPos(d1));
        Action d3 = deliverSpecimen(drive.findEndPos(d2));

        Action park = drive.actionBuilder(drive.findEndPos(d3))
                .splineToSplineHeading(transform(24 ,-24, Math.PI*3/2), xformHeading(Math.PI*1/2)).build();

        return new SequentialAction(
                d1,     // deliver preload, pick up 1st sample and take to  human to make specimen
                d2,     // deliver 2nd specimena dn pick up sample and have human make specimen
                park    // no time to deliver, go park.
        );
    }

}
