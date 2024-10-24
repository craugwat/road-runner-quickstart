package com.example.trajectoryactions.SimConfig;


//import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class SimMecanumDrive implements Drive {
    public static class Params {
        // drive model parameters
        public double inPerTick = 0.00297882243425645799394926693042;  //  0.00303797;
        public double lateralInPerTick = inPerTick; // not used with deadwheel?  but can't be zero or get uncaught exception.   0.00031215658162729576;
        public double trackWidthTicks = 4880.0499529207445; //3894.7640479958563;  Is this used with deadwheels?

        // path profile parameters (in inches)
        public double maxWheelVel = 60;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 60;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 5.0;

        public double lateralGain = 5.0;
        public double headingGain = 7.0; // shared with turn

        public double axialVelGain = 0.01;
        public double lateralVelGain = 1;
        public double headingVelGain = 0.01; // shared with turn
    }

    public static Params PARAMS = new Params();


    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);


    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);


    public Pose2d pose;
    private String robotColor = "#4CAF50";

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();


    public SimMecanumDrive(Pose2d pose) {
        this.pose = pose;

    }

    public SimMecanumDrive(Pose2d pose, String robotColor) {
        this.pose = pose;
        this.robotColor = robotColor;

    }

    public void setPose(Pose2d p) {this.pose = p;}; // CAW added

    public Pose2d getPose() {return this.pose;}
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private Pose2d endPose; // CAW added

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    (int) Math.ceil(t.path.length() / 2));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];


            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }

            endPose = t.path.get(t.path.length(),1).value();// CAW added
        }

        // CAW added this method to get the last postion,
        // note this may not work in a trejectorySequence where initial Actions is smaller than total number of actions.
        // I'm still learning roadrunner and not sure when there ar emore actions after initial actions?
        public Pose2d getEndPos(){  // CAW added
            return endPose;
        }

        @Override
        public boolean run(/*@NonNull*/ TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            pose = txWorldTarget.value(); // this line is our localizer for simulator!  Set robot position to target postion!

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()));

//            Pose2d error = txWorldTarget.value().minusExp(pose);
//            p.put("xError", error.position.x);
//            p.put("yError", error.position.y);
//            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(/*@NonNull*/ TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            pose = txWorldTarget.value(); // this line is our localizer for simulator!  Set robot position to target postion!

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    // in simulator class the physical robot does not move
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d retPost = new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        return retPost;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.setStroke(robotColor);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    private Vector2d rotateVector(Vector2d pt, Rotation2d angle)
    {
       return new Vector2d(pt.x * angle.real - pt.y*angle.imag, pt.x * angle.imag + pt.y*angle.real);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }


    // this method checks for class types before casting to reach the functions we need.
    // returns the last robot position expected.
    // note it will return null and if no move commands are found in the action list.
    // this may cause an exception in a trajectory that builds with startpos of null.
    public Pose2d findEndPos(Action a){
        Pose2d endPos = null;
        if (a instanceof SequentialAction) {
            List<Action> actList = ((SequentialAction) a).getInitialActions();
            // start at last action in list and work backwards looking for a robot motion
            int index = ((SequentialAction) a).getInitialActions().size() - 1;
            while (endPos == null && index >= 0) {
                Action lastAct = actList.get(index);
                if (lastAct instanceof SimMecanumDrive.FollowTrajectoryAction) {
                    endPos = ((FollowTrajectoryAction) lastAct).getEndPos();
                }
                if (lastAct instanceof SequentialAction) {
                    return findEndPos(lastAct);
                }
                if (lastAct instanceof TurnAction) {
                    Pose2d pos = ((TurnAction) lastAct).turn.beginPose;
                    endPos = new Pose2d(pos.position.x, pos.position.y, pos.heading.log() + ((TurnAction) lastAct).turn.angle);
                }
                index--;
            }
        }
        if (endPos == null)
            endPos = this.pose;  // if actions do not move robot return our initial position = end position.
        return endPos;
    }


}
