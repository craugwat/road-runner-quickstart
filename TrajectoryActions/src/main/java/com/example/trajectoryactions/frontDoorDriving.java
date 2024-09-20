package com.example.trajectoryactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.example.trajectoryactions.SimConfig.Drive;


public class frontDoorDriving extends wgwABCommon{
    public frontDoorDriving(Drive d) {
        super(d);
    }


    // sample action
    // this one is used by our simulator instead of moving motors/servos etc.

    // these are similar overall path to far but adding in facing toward the backdrop to detect
    // robots and then turning afterward to deliver

    //How we park on the diagonal side since we came from the far side
    public Action parkFar(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(44,14 * dir, Math.toRadians(180 * dir)), Math.toRadians(270 * dir))
                .build());
    }

    //Goes to the wait location from the center for the robots to be detected
    public Action wallDriveToRobotDetect(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        TranslationalVelConstraint translationalVelConstraint = new TranslationalVelConstraint(6.0);
        return (trajActBuilder
                .setTangent(Math.toRadians(90 * dir))
                .splineToLinearHeading(new Pose2d(-42,56 * dir,Math.toRadians(270 * dir)),Math.toRadians(0 * dir), translationalVelConstraint)
                .splineToSplineHeading(new Pose2d(-36, 36 * dir,Math.toRadians(270 * dir)), Math.toRadians(270 * dir), translationalVelConstraint)
                .splineToSplineHeading(new Pose2d(-36, 24 * dir,Math.toRadians(270 * dir)), Math.toRadians(270 * dir), translationalVelConstraint)
                .splineToSplineHeading(new Pose2d(12,12 * dir,Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(robotDetectX,12 * dir,Math.toRadians(90 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    // aprilTagBlueFarCenter takes you from the checking for robot pos to apriltag
    public Action aprilTagWallSideRD(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(backDropX, 42 * dir, Math.toRadians(180 * dir)), Math.toRadians(90 * dir))
                .build());
    }

    // aprilTagRedFar takes you from the checking for robot pos to apriltag
    public Action aprilTagFieldSideRD(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .turnTo(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(backDropX,30 * dir,Math.toRadians(180 * dir)),Math.toRadians(90 * dir))
                .build());
    }

    //   ______
    //  | - -  |
    //   | ~  |
    // --  |   --
    //  (      )

    // aprilTagBlueFarCenter takes you from the checking for robot pos to apriltag
    public Action aprilTagCenterRD (Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .turnTo(Math.toRadians(180 * dir))
                .setTangent(Math.toRadians(90 * dir))
                .splineToLinearHeading(new Pose2d(backDropX,36 * dir,Math.toRadians(180 * dir)),Math.toRadians(90 * dir))
                .build());
    }

    // Goes to the spikemark underneath the rigging on the blue far side
    public Action spikeFarWallSideRD(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-47.5,42 * dir,Math.toRadians(270 * dir)),Math.toRadians(270 * dir))
                .build());
    }

    // spikeFarRiggingSide is the first move of this path - start pos to spike closer to rigging on the far side
    public Action spikeFarRiggingSideRD(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-44, 48 * dir, Math.toRadians(315 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(-34, 36 * dir, Math.toRadians(345 * dir)), Math.toRadians(335 * dir))
                .build());
    }

    // spikeBlueFarCenter is the first move of this path - start pos to center spike on blue far side
    public Action spikeFarCenterRD(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-56, 46 * dir, Math.toRadians(0 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(-47, 24 * dir, Math.toRadians(0 * dir)), Math.toRadians(335 * dir))
                .build());
    }

    public Action centerDriveToRobotDetect(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(135 * dir))
                .splineToLinearHeading(new Pose2d(-56, 16 * dir, Math.toRadians(0 * dir)), Math.toRadians(270 * dir))
                .splineToSplineHeading(new Pose2d(-34, 12 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(10, 12 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(robotDetectX, 12 * dir, Math.toRadians(90 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    public Action riggingDriveToRobotDetect(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-52,16 * dir, Math.toRadians(0 * dir)), Math.toRadians(270 * dir))
                //.waitSeconds(8)
                .splineToSplineHeading(new Pose2d(-28,12 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
             //   .splineToSplineHeading(new Pose2d(-10, 8 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(12, 12 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(robotDetectX, 12 * dir, Math.toRadians(90 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    public SequentialAction farDoor(Drive drive, Action dropPurple, Action dropYellow,
                                Action liftStateMachineDeliverAction, Action liftStateMachineCollectAction,
                                    FieldSide fieldSide, SelectedSpike selectedSpike, Action tagDetection,
                                Action waitForRobotAction, Action clawSwitchAction)
    {
        SequentialAction retVal;
//
//     (-_-)
//      /|\
//       |
//      / \
        Action spikeAction;
        Action driveToWaitAction = null;
        //Action waitForRobotAction = new DelayAction("waiting for robot", 10.0);
        Action driveToBackDropAction = null;
        Action moveTowardBackDrop;
        Action parkAction;
        Action backAwayFromBackDrop;

        if (selectedSpike == SelectedSpike.NONE)
        {
            selectedSpike = SelectedSpike.LEFT;
        }

        // use spike mark selection to decide between first two paths, then paths are same
        // note that left spike on blue and red is different path
        if ((selectedSpike == SelectedSpike.MIDDLE) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeAction             = spikeFarCenterRD(drive, startPosA2, fieldSide);
            driveToWaitAction       = centerDriveToRobotDetect(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagCenterRD(drive, drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.MIDDLE) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeAction             = spikeFarCenterRD(drive, startPosF2, fieldSide);
            driveToWaitAction       = centerDriveToRobotDetect(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagCenterRD(drive, drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeAction             = spikeFarRiggingSideRD(drive, startPosA2, fieldSide);
            driveToWaitAction       = riggingDriveToRobotDetect(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagWallSideRD(drive, drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeAction             = spikeFarWallSideRD(drive, startPosF2, fieldSide);
            driveToWaitAction       = wallDriveToRobotDetect(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagFieldSideRD(drive, drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.RIGHT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeAction             = spikeFarWallSideRD(drive, startPosA2, fieldSide);
            driveToWaitAction       = wallDriveToRobotDetect(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagFieldSideRD(drive, drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else // RIGHT, RED
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeAction             = spikeFarRiggingSideRD(drive, startPosF2, fieldSide);
            driveToWaitAction       = riggingDriveToRobotDetect(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagWallSideRD(drive, drive.findEndPos(driveToWaitAction), fieldSide);

        }

        // build the separate trajectories for this path specifically
        moveTowardBackDrop       = backAwayFromBackDrop(drive.findEndPos(driveToBackDropAction), -4.5);
        backAwayFromBackDrop     = backAwayFromBackDrop(drive.findEndPos(moveTowardBackDrop), +4.5);
        parkAction               = parkFar(drive, drive.findEndPos(backAwayFromBackDrop), fieldSide);



        // bring all those trajectories together into a SequentialAction
        retVal = new SequentialAction(
                spikeAction,
                dropPurple,
                driveToWaitAction,
                clawSwitchAction,
                waitForRobotAction,
                new ParallelAction(driveToBackDropAction, liftStateMachineDeliverAction),
                //tagDetection,
                new driveToX(drive, 4),  //moveTowardBackDrop,
                dropYellow,
                new driveToX(drive, -4),
                new ParallelAction(
                        parkAction,
                        liftStateMachineCollectAction)
                // these are for when we start getting more white pixels instead of parking
//               new ParallelAction(
//                    new SequentialAction(
//                        new driveToX(drive, -4),
//                        backdropToWhitePixel),
//                liftStateMachineCollectAction));
        );

        return retVal;
    }
}


