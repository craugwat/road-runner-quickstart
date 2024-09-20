package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;


public class wgwABNearSideDriving extends wgwABCommon{

    wgwABBackdropToPixel backdropToPixelDriving = new wgwABBackdropToPixel();

    public wgwABNearSideDriving(Drive d) {
        super(d);
    }

    // spikeBlueCloseCenter is the first move of this path - start pos to center spike on blue close side
    public Action spikeCloseCenter(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(32, 46 * dir, Math.toRadians(180 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(24, 24.5 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .build());
    }

    // spikeCloseBackdropSide is the first move of this path - start pos to spike towards backdrop
    public Action spikeCloseBackdropSide(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(23.5,40 * dir,Math.toRadians(270 * dir)),Math.toRadians(270 * dir))
                .build());
    }

    // spikeCloseRiggingSide is the first move of this path - start pos to spike closer to rigging
    public Action spikeCloseRiggingSide(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(20, 48 * dir, Math.toRadians(225 * dir)), Math.toRadians(270 * dir))
                .splineToLinearHeading(new Pose2d(7.5, 34 * dir, Math.toRadians(202.5 * dir)), Math.toRadians(180 * dir))
                .build());
    }


    public SequentialAction close(Drive drive, Action dropPurple, Action dropYellow,
                                  Action liftStateMachineDeliverAction,
                                  Action liftStateMachineCollectAction,
                                  FieldSide fieldSide,
                                  SelectedSpike selectedSpike, Action tagDetection,
                                  ParkingPosState parkingPos)
    {
        SequentialAction retVal;

        Action spikeAction;
        Action driveToBackDropAction;
        Action moveTowardBackDrop;
        Action parkAction;
        Action backAwayFromBackDrop;
        Action backdropToWhitePixel;

        if (selectedSpike == SelectedSpike.NONE)
        {
            selectedSpike = SelectedSpike.LEFT;
        }

        // use spike mark selection to decide between first two paths, then paths are same
        // note that left spike on blue and red is different path
        if ((selectedSpike == SelectedSpike.MIDDLE) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA4); // set the robot's actual start position!
            spikeAction             = spikeCloseCenter(drive, startPosA4, fieldSide);
            driveToBackDropAction   = aprilTagCenter(drive.findEndPos(spikeAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.MIDDLE) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF4); // set the robot's actual start position!
            spikeAction             = spikeCloseCenter(drive, startPosF4, fieldSide);
            driveToBackDropAction   = aprilTagCenter(drive.findEndPos(spikeAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA4); // set the robot's actual start position!
            spikeAction             = spikeCloseBackdropSide(drive, startPosA4, fieldSide);
            driveToBackDropAction   = aprilTagWallSide(drive.findEndPos(spikeAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.LEFT) && (fieldSide == FieldSide.RED))
        {
            drive.setPose(startPosF4); // set the robot's actual start position!
            spikeAction             = spikeCloseRiggingSide(drive, startPosF4, fieldSide);
            driveToBackDropAction   = aprilTagFieldSide(drive.findEndPos(spikeAction), fieldSide);
        }
        else if ((selectedSpike == SelectedSpike.RIGHT) && (fieldSide == FieldSide.BLUE))
        {
            drive.setPose(startPosA4); // set the robot's actual start position!
            spikeAction             = spikeCloseRiggingSide(drive, startPosA4, fieldSide);
            driveToBackDropAction   = aprilTagFieldSide(drive.findEndPos(spikeAction), fieldSide);
        }
        else // RIGHT, RED
        {
            drive.setPose(startPosF4); // set the robot's actual start position!
            spikeAction             = spikeCloseBackdropSide(drive, startPosF4, fieldSide);
            driveToBackDropAction   = aprilTagWallSide(drive.findEndPos(spikeAction), fieldSide);

        }



        // build the separate trajectories for this path specifically
        moveTowardBackDrop       = backAwayFromBackDrop(drive.findEndPos(driveToBackDropAction), -5.5);
        backAwayFromBackDrop     = backAwayFromBackDrop(drive.findEndPos(moveTowardBackDrop), +5.5);
        if (parkingPos == ParkingPosState.MIDDLE){
            parkAction           = parkMiddle(drive, drive.findEndPos(backAwayFromBackDrop), fieldSide);
        } else {
            parkAction           = parkCorner(drive, drive.findEndPos(backAwayFromBackDrop), fieldSide);
        }
        backdropToWhitePixel     = backdropToPixelDriving.backdropToWhitePixelTruss(drive, drive.findEndPos(backAwayFromBackDrop), fieldSide);

        // bring all those trajectories together into a SequentialAction
        retVal = new SequentialAction(
                spikeAction,
                dropPurple,
                new ParallelAction(driveToBackDropAction, liftStateMachineDeliverAction),
                //tagDetection,
                new driveToX(drive, 4),  //moveTowardBackDrop,
                dropYellow,
                new driveToX(drive, -4),
                new ParallelAction(
                        parkAction,
                        liftStateMachineCollectAction));
        // these are for when we start getting more white pixels instead of parking
//               new ParallelAction(
//                    new SequentialAction(
//                        new driveToX(drive, -4),
//                        backdropToWhitePixel),
//                liftStateMachineCollectAction));
        return retVal;
    }

}


