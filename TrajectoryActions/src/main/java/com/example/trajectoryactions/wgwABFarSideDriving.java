package com.example.trajectoryactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;

public class wgwABFarSideDriving extends wgwABCommon{

    public wgwABFarSideDriving(Drive d) {
        super(d);
    }

    public SequentialAction far(Drive drive, Action dropPurple, Action dropYellow,
                                Action liftStateMachineDeliverAction,
                                Action liftStateMachineCollectAction,
                                FieldSide fieldSide,
                                SelectedSpike selectedSpike, Action tagDetection,
                                Action waitForRobotAction, Action clawSwitchAction)
    {
        SequentialAction retVal;

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
            spikeAction             = spikeFarCenter(drive, startPosA2, fieldSide);
            driveToWaitAction       = centerDriveToWaitLocation(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagCenter(drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.MIDDLE) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeAction             = spikeFarCenter(drive, startPosF2, fieldSide);
            driveToWaitAction       = centerDriveToWaitLocation(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagCenter(drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeAction             = spikeFarRiggingSide(drive, startPosA2, fieldSide);
            driveToWaitAction       = riggingDriveToWaitLocation(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagWallSide(drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeAction             = spikeFarWallSide(drive, startPosF2, fieldSide);
            driveToWaitAction       = wallDriveToWaitLocation(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagFieldSide(drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.RIGHT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA2); // set the robot's actual start position!
            spikeAction             = spikeFarWallSide(drive, startPosA2, fieldSide);
            driveToWaitAction       = wallDriveToWaitLocation(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagFieldSide(drive.findEndPos(driveToWaitAction), fieldSide);
        }
        else // RIGHT, RED
        {
            drive.setPose(startPosF2); // set the robot's actual start position!
            spikeAction             = spikeFarRiggingSide(drive, startPosF2, fieldSide);
            driveToWaitAction       = riggingDriveToWaitLocation(drive, drive.findEndPos(spikeAction), fieldSide);
            driveToBackDropAction   = aprilTagWallSide(drive.findEndPos(driveToWaitAction), fieldSide);

        }

        // build the separate trajectories for this path specifically
        moveTowardBackDrop       = backAwayFromBackDrop(drive.findEndPos(driveToBackDropAction), -4.5);
        backAwayFromBackDrop     = backAwayFromBackDrop(drive.findEndPos(moveTowardBackDrop), +4.5);
        parkAction               = parkMiddle(drive, drive.findEndPos(backAwayFromBackDrop), fieldSide);

        // bring all those trajectories together into a SequentialAction
        retVal = new SequentialAction(
                spikeAction,
                dropPurple,
                driveToWaitAction,
                clawSwitchAction,
                new ParallelAction(waitForRobotAction, liftStateMachineDeliverAction),
                // waitForRobotAction,
                driveToBackDropAction,
                tagDetection,
                // liftStateMachineDeliverAction,
                new driveToX(drive, 4),  //moveTowardBackDrop,
                dropYellow,
                new ParallelAction(new SequentialAction(new driveToX(drive, -4), parkAction), liftStateMachineCollectAction)
//                new driveToX(drive, -4),  //backAwayFromBackDrop,
//                liftStateMachineCollectAction,
//                parkAction
        );
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

}


