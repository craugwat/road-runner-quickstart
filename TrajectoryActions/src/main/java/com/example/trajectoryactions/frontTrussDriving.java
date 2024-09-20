package com.example.trajectoryactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;


public class frontTrussDriving extends wgwABCommon{


    public frontTrussDriving(Drive d) {
        super(d);
    }

    // Goes to the spikemark underneath the rigging on the blue far side
    public Action spikeFarWallSideTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-47.5,42 * dir,Math.toRadians(270 * dir)),Math.toRadians(270 * dir))
                .build());
    }

    // spikeFarRiggingSide is the first move of this path - start pos to spike closer to rigging on the far side
    public Action spikeFarRiggingSideTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
//                .lineToYSplineHeading(48*dir,Math.toRadians(90 * dir))
                .splineToLinearHeading(new Pose2d(startPos.position.x, 48 * dir, Math.toRadians(315 * dir)), Math.toRadians(270 * dir))
                .splineToSplineHeading(new Pose2d(-32.5, 37 * dir, Math.toRadians(345 * dir)), Math.toRadians(337.5 * dir))
                .build());

    }
    public Action riggingDriveToWaitLocationTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        TranslationalVelConstraint translationalVelConstraint = new TranslationalVelConstraint(6.0);
        return (trajActBuilder
                .setTangent(Math.toRadians(90*dir))

//                .strafeToSplineHeading(new Vector2d(-44,42 * dir),Math.toRadians(0 * dir))
                .splineToLinearHeading(new Pose2d(-48,50 * dir, Math.toRadians(0 * dir)), Math.toRadians(90 * dir))
                .splineToLinearHeading(new Pose2d(-20,60 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(35,60 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(robotDetectX,60 * dir,Math.toRadians(270 * dir)), Math.toRadians(0 * dir))
                .build());

    }
    // spikeBlueFarCenter is the first move of this path - start pos to center spike on blue far side
    public Action spikeFarCenterTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-40, 34 * dir, Math.toRadians(270 * dir)), Math.toRadians(270 * dir))
                .build());
    }

    //Goes to the wait location from the center for the robots to be detected
    public Action wallDriveToWaitLocationTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-40,60 * dir,Math.toRadians(0 * dir)),Math.toRadians(0 * dir))
                .setTangent(Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(12,60 * dir,Math.toRadians(0 * dir)),Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(25,60 * dir,Math.toRadians(0 * dir)),Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(robotDetectX,60 * dir,Math.toRadians(270 * dir)),Math.toRadians(0 * dir))
                .build());
    }

    public Action centerDriveToWaitLocationTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                //  .splineToLinearHeading(new Pose2d(-56,16 * dir,Math.toRadians(270 * dir)), Math.toRadians(270 * dir))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-34, 60 * dir,Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(10,60 * dir,Math.toRadians(0 * dir)),Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(16,60 * dir,Math.toRadians(0 * dir)),Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(robotDetectX,60 * dir,Math.toRadians(270 * dir)),Math.toRadians(0 * dir))
                .build());
    }

    //How we park on the diagonal side since we came from the far side
//    public Action parkFarTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
//        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
//        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
//        return (trajActBuilder
//                .setReversed(false)
//                //.splineToLinearHeading(new Pose2d(44,14 * dir, Math.toRadians(180 * dir)), Math.toRadians(270 * dir))
//                .setReversed(true)
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(58,10 * dir, Math.toRadians(180)), Math.toRadians(0 * dir))
//                .build());
//    }
    public Action parkFarTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                //.splineToLinearHeading(new Pose2d(44,50 * dir, Math.toRadians(180 * dir)), Math.toRadians(90 * dir))
                .setTangent(Math.toRadians(90*dir))
                .splineToLinearHeading(new Pose2d(55,62 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    // aprilTagRedFar takes you from the checking for robot pos to apriltag
    public Action aprilTagFieldSideTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(backDropX,30.0 * dir,Math.toRadians(180 * dir)),Math.toRadians(270 * dir))
                .build());
    }
    // aprilTagRedFar takes you from the checking for robot pos to apriltag
    public Action aprilTagCenterTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(backDropX,36 * dir,Math.toRadians(180 * dir)),Math.toRadians(270 * dir))
                .build());
    }
    // aprilTagRedFar takes you from the checking for robot pos to apriltag
    public Action aprilTagWallSideTrusses(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(backDropX,42 * dir,Math.toRadians(180 * dir)),Math.toRadians(270 * dir))
                .build());
    }


    public SequentialAction farTrusses(Drive drive, Action dropPurple, Action dropYellow,
                                       Action liftStateMachineDeliverAction, Action liftStateMachineCollectAction,
                                       FieldSide fieldSide, SelectedSpike selectedSpike, Action tagDetection,
                                       Action waitForRobotAction, Action clawSwitchAction)
    {
        SequentialAction retVal;

        Action spikeActionTrusses;
        Action driveToWaitActionTrusses =  null;
        //Action waitForRobotAction = new DelayAction("waiting for robot", 10.0);
        Action driveToBackDropActionTrusses = null;
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
            spikeActionTrusses             = spikeFarCenterTrusses(drive, startPosA2, fieldSide);
            driveToWaitActionTrusses       = centerDriveToWaitLocationTrusses(drive, drive.findEndPos(spikeActionTrusses), fieldSide);
            driveToBackDropActionTrusses   = aprilTagCenterTrusses(drive, drive.findEndPos(driveToWaitActionTrusses), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.MIDDLE) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeActionTrusses             = spikeFarCenterTrusses(drive, startPosF2, fieldSide);
            driveToWaitActionTrusses       = centerDriveToWaitLocationTrusses(drive, drive.findEndPos(spikeActionTrusses), fieldSide);
            driveToBackDropActionTrusses   = aprilTagCenterTrusses(drive, drive.findEndPos(driveToWaitActionTrusses), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeActionTrusses             = spikeFarRiggingSideTrusses(drive, startPosA2, fieldSide);
            driveToWaitActionTrusses       = riggingDriveToWaitLocationTrusses(drive, drive.findEndPos(spikeActionTrusses), fieldSide);
            driveToBackDropActionTrusses   = aprilTagWallSideTrusses(drive, drive.findEndPos(driveToWaitActionTrusses), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeActionTrusses             = spikeFarWallSideTrusses(drive, startPosF2, fieldSide);
            driveToWaitActionTrusses       = wallDriveToWaitLocationTrusses(drive, drive.findEndPos(spikeActionTrusses), fieldSide);
            driveToBackDropActionTrusses   = aprilTagFieldSideTrusses(drive, drive.findEndPos(driveToWaitActionTrusses), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.RIGHT) && (fieldSide == FieldSide.BLUE)) {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeActionTrusses = spikeFarWallSideTrusses(drive, startPosA2, fieldSide);
            driveToWaitActionTrusses = wallDriveToWaitLocationTrusses(drive, drive.findEndPos(spikeActionTrusses), fieldSide);
            driveToBackDropActionTrusses = aprilTagFieldSideTrusses(drive, drive.findEndPos(driveToWaitActionTrusses), fieldSide);
        }
        else // RIGHT, RED
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeActionTrusses             = spikeFarRiggingSideTrusses(drive, startPosF2, fieldSide);
            driveToWaitActionTrusses       = riggingDriveToWaitLocationTrusses(drive, drive.findEndPos(spikeActionTrusses), fieldSide);
            driveToBackDropActionTrusses   = aprilTagWallSideTrusses(drive, drive.findEndPos(driveToWaitActionTrusses), fieldSide);

        }

//     build the separate trajectories for this path specifically
        moveTowardBackDrop       = backAwayFromBackDrop(drive.findEndPos(driveToBackDropActionTrusses), -4.5);
        backAwayFromBackDrop     = backAwayFromBackDrop(drive.findEndPos(moveTowardBackDrop), +4.5);
        parkAction               = parkCorner(drive, drive.findEndPos(backAwayFromBackDrop), fieldSide);


        // bring all those trajectories together into a SequentialAction
        retVal = new SequentialAction(
                spikeActionTrusses,
                dropPurple,
                driveToWaitActionTrusses,
                clawSwitchAction,
                waitForRobotAction,
                new ParallelAction(driveToBackDropActionTrusses, liftStateMachineDeliverAction),
                tagDetection,
                new driveToX(drive, 4),
                dropYellow,
                new driveToX(drive, -4),
                new ParallelAction(
                        parkAction,
                        liftStateMachineCollectAction));
        return retVal;
    }

    // Goes to the spikemark underneath the rigging on the blue far side
    public Action spikeFarWallSide(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-47.5,42 * dir,Math.toRadians(270 * dir)),Math.toRadians(270 * dir))
                .build());
    }

    // spikeFarRiggingSide is the first move of this path - start pos to spike closer to rigging on the far side
    public Action spikeFarRiggingSide(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-44, 48 * dir, Math.toRadians(315 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(-32.5, 36 * dir, Math.toRadians(345 * dir)), Math.toRadians(337.5 * dir))
                .build());

    }
    public Action riggingDriveToWaitLocation(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-40,27 * dir,Math.toRadians(325 * dir)),Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(-40,10 * dir,Math.toRadians(180 * dir)), Math.toRadians(270 * dir))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16,10 * dir),Math.toRadians(0 * dir))
                .splineToLinearHeading(new Pose2d(24,11 * dir,Math.toRadians(180 * dir)),Math.toRadians(0 * dir))
                .build());

    }
    // spikeBlueFarCenter is the first move of this path - start pos to center spike on blue far side
    public Action spikeFarCenter(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-56, 46 * dir, Math.toRadians(0 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(-47, 22 * dir, Math.toRadians(0 * dir)), Math.toRadians(335 * dir))
                .build());
    }

    //Goes to the wait location from the center for the robots to be detected
    public Action wallDriveToWaitLocation(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-34,36 * dir,Math.toRadians(180 * dir)),Math.toRadians(270 * dir))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-34,10 * dir), Math.toRadians(180 * dir))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(24,11 * dir,Math.toRadians(180 * dir)),Math.toRadians(0 * dir))
                .build());
    }

    public Action centerDriveToWaitLocation(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .lineToX(-56)
                .splineToLinearHeading(new Pose2d(-54, 16 * dir,Math.toRadians(270 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(-34,11 * dir,Math.toRadians(180 * dir)),Math.toRadians(0 * dir))
                .splineToLinearHeading(new Pose2d(24,11 * dir,Math.toRadians(180 * dir)),Math.toRadians(0 * dir))
                .build());
    }

    //How we park on the diagonal side since we came from the far side
    public Action parkFar(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(270 * dir))
                .lineToY(14 * dir)
                .build());
    }
}


