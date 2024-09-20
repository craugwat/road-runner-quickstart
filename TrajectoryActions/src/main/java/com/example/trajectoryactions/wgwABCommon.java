package com.example.trajectoryactions;

import static com.acmerobotics.roadrunner.Actions.now;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;

import org.jetbrains.annotations.NotNull;

public class wgwABCommon {
    static public Pose2d startPosA2 = new Pose2d(-40,  63.5, Math.toRadians(270));
    static public Pose2d startPosA4 = new Pose2d( 16,  63.5, Math.toRadians(270));
    static public Pose2d startPosF2 = new Pose2d(-40, -63.5, Math.toRadians(90));
    static public Pose2d startPosF4 = new Pose2d( 16, -63.5, Math.toRadians(90));
    static public Pose2d startCenterField = new Pose2d(0, 0, Math.toRadians(0));

    public enum FieldSide {RED, BLUE}
    public enum Direction {LEFT, RIGHT}
    public enum SelectedSpike {NONE, LEFT, MIDDLE, RIGHT}
    public enum ParkingPosState {CORNER, MIDDLE, UNKNOWN}


    private Drive drive = null;  // this was static but that won't work with multiple robots in simulator

    // The common back drop postion for the X value
    public static int backDropX = 42;
    public static int robotDetectX = 44;



    public wgwABCommon(Drive d){
        drive = d;
    }

    // sample action
    // this one is used by our simulator instead of moving motors/servos etc.
    class  SimTimedAction implements Action  {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt;        // total time for this action to run
            private final String msg;

            SimTimedAction(String message, double deltaTime) {
                msg = message;
                dt = deltaTime;
            }

            @Override
            public boolean run(TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                String formatedStr = String.format(msg + " %.2f of ", t); //hijacking to send 2 vals
                p.put(formatedStr, dt);
                return t < dt;          // actions are run until they return false;
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template reequired it to.
        }


    class driveToX implements Action{
        private boolean firstRun = true;
        private Drive drive;
        private double distance;
        private Action action;
        public driveToX(Drive drive, double distance){
            this.drive = drive;
            this.distance = distance;
        }
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            if (firstRun){
                firstRun = false;
                Pose2d pose = drive.getPose();
                TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(pose);
                action = trajActBuilder
                        .lineToX((pose.position.x + distance))
                        .build();
            }
            return action.run(telemetryPacket);
        }
    }

    // needs to be in this or other common class accessible by robot & simulator
    // initialize the actions to simulator actions. robot.init can change these to robot actions
    public class actionParameters {
        public Action dropPurple = new SimTimedAction("Drop Purple", 1.0);
        public Action dropYellow = new SimTimedAction("Drop Yellow", 0.5);
        public Action tagDetection =  new SimTimedAction("tag driving", 1.0);
        public Action liftStateMachineDeliverAction = new SimTimedAction("state machine moving to deliver", 2.0);
        public Action liftStateMachineCollectAction = new SimTimedAction("state machine moving to deliver", 2.0);
        public Action waitForRobot = new SimTimedAction("Waiting for partner robot", 2.0);
        public Action clawSwitchAction = new SimTimedAction("switching the pixel", 1);
        public FieldSide fieldSide = FieldSide.BLUE;
        public SelectedSpike selectedSpike =SelectedSpike.NONE;
        public ParkingPosState parkingPos;
    }

    double beginGameTimes = -1.0;  // timer to track when we started
    public void startGameTimer(){
        beginGameTimes = now();
    }

    // first call to the run method starts the timer, but returns false so action will not be called again
    // future calls to run will return true until set game time has been reached.
    class  GameTimerAction implements Action  {
        private double t = 0.0;         // time this action has been running
        private double dt;        // total time for this action to run

        GameTimerAction(double deltaTime) {
            dt = deltaTime;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            if(beginGameTimes < 0) {        // Game timers should have already been started
                return false;               // bail out they did not start game timer
            }
            t = now()-beginGameTimes;  // how long have we been running
            String formatedStr = String.format("Sleep until game time %.2f of ", t); //hijacking to send 2 vals
            p.put(formatedStr, dt);
            return t < dt;          // actions are run until they return false;
        }

        @Override
        public void preview(Canvas c) {}  // not used, but template reequired it to.
    }

    /***** student trajectories below here *********************************************************/
//    // empty method to override for nearside class
//    public SequentialAction close(Action dropPurple, Action dropYellow,
//                                  Action liftStateMachineDeliverAction, Action liftStateMachineCollectAction,
//                                  FieldSide fieldSide, SelectedSpike selectedSpike, Action tagDetection)
//    {
//        SequentialAction retVal;
//        retVal = new SequentialAction();
//        return retVal;
//    }
//
//    // empty method to override for farside class
//    public SequentialAction far(Action dropPurple, Action dropYellow,
//                                Action liftStateMachineDeliverAction, Action liftStateMachineCollectAction,
//                                FieldSide fieldSide, SelectedSpike selectedSpike, Action tagDetection,
//                                Action waitForRobotAction, Action clawSwitchAction)
//    {
//        SequentialAction retVal;
//        retVal = new SequentialAction();
//        return retVal;
//    }

    // aprilTagBlueFarCenter takes you from the checking for robot pos to apriltag
    Action aprilTagCenter (Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(backDropX,36 * dir,Math.toRadians(180 * dir)),Math.toRadians(0 * dir))
                .build());
    }

    Action backAwayFromBackDrop(Pose2d startPos, double distance)
    {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        return ( trajActBuilder
                .setReversed(false)
                .lineToX(startPos.position.x-distance)
                .build());
    }

    // aprilTagBlueFarCenter takes you from the checking for robot pos to apriltag
    Action aprilTagWallSide(Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(backDropX, 42 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    // aprilTagRedFar takes you from the checking for robot pos to apriltag
    Action aprilTagFieldSide(Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(backDropX,30 * dir,Math.toRadians(180 * dir)),Math.toRadians(0 * dir))
                .build());
    }

    ParallelAction timers()
    {
        ParallelAction retVal;
        retVal = new ParallelAction(
                new SimTimedAction("this is timer 1", 7),
                new SimTimedAction("this is timer 2", 13)
        );
        return retVal;
    }

    Action parkCorner(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                //.splineToLinearHeading(new Pose2d(44,50 * dir, Math.toRadians(180 * dir)), Math.toRadians(90 * dir))
                .setTangent(Math.toRadians(90*dir))
                .splineToLinearHeading(new Pose2d(55,62 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    Action parkMiddle(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(270 * dir))
                .lineToY(14 * dir)
                .build());
    }
}


