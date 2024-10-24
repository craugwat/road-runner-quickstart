package com.example.trajectoryactions.Samples;

import static com.acmerobotics.roadrunner.Actions.now;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.example.trajectoryactions.SimConfig.Drive;
import com.example.trajectoryactions.SimConfig.SimMecanumDrive;

public class commonTrajectories {
    static public Pose2d startPosA2 = new Pose2d(-40,  63.5, Math.toRadians(270));
    static public Pose2d startPosA4 = new Pose2d( 16,  63.5, Math.toRadians(270));
    static public Pose2d startPosF2 = new Pose2d(-36, -63.5, Math.toRadians(90));
    static public Pose2d startPosF4 = new Pose2d( 16, -63.5, Math.toRadians(270));

    public enum FieldSide {RED, BLUE}

    Drive drive = null;  // this was static but that won't work with multiple robots in simulator

    public commonTrajectories(Drive d){
        drive = d;
    }

    public Pose2d transform(double x, double y, double heading){
        return transform(new Pose2d(x,y,heading));
    }

    public Pose2d transform(Pose2d pose){
        if (actionParameters.fieldSide == FieldSide.BLUE)
            return new Pose2d(pose.position.unaryMinus(), pose.heading.toDouble() - Math.PI);
        else
            return pose;
    }

    public double xformHeading(double heading){
        if (actionParameters.fieldSide == FieldSide.BLUE)
            return heading - Math.PI;
        else
            return heading;
    }

    // sample action
    // this one is used by our simulator instead of moving motors/servos etc.
    class  SimTimedAction implements Action {
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

            boolean retVal = t < dt;    // reset so timer can be run again
            if (! retVal){
                beginTs = -1;
                t = 0;
            }

            return retVal;          // actions are run until they return false;
        }

        @Override
        public void preview(Canvas c) {}  // not used, but template reequired it to.
    }

    // needs to be in this or other common class accessible by robot & simulator
    // initialize the actions to simulator actions. robot.init can change these to robot actions
    public class ActionParameters {
        public Action collectSample = new SimTimedAction("Collect Sample", 1.0);
        public Action deliverSample = new SimTimedAction("Deliver Sample", 1.0);
        public Action CollectSpecimen =  new SimTimedAction("Collect Specimen", 1.0);
        public Action deliverSpecimen = new SimTimedAction("Deliver Specimen", 1.0);
        public Action liftUp = new SimTimedAction("Lift UP", 2.0);
        public Action liftDown = new SimTimedAction("Lift Down", 2.0);
        public FieldSide fieldSide = FieldSide.BLUE;
    }
    public ActionParameters actionParameters = new ActionParameters();

}
