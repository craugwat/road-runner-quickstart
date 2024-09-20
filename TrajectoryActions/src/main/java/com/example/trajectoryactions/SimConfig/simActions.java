package com.example.trajectoryactions.SimConfig;

import static com.acmerobotics.roadrunner.Actions.now;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class simActions {
    public Supplier<Action> dropPurple =            () -> new SimTimedAction("Drop Purple", 1.0);
    public Supplier<Action> dropYellow =            () -> new SimTimedAction("Drop Yellow", 0.5);
    public Supplier<Action> pixelDropperInit =      () -> new SimTimedAction("PixelDropper2init", 0.5);
    public Supplier<Action> tagAction =             () -> new SimTimedAction("tag driving", 1.0);
    public Supplier<Action> stateMachineAction =    () -> new SimTimedAction("state machine moving to deliver", 2.0);
    public Supplier<Action> waitForRobotAction =    () -> new SimTimedAction("Waiting for partner robot", 5.0);
    public Supplier<Action> clawSwitchAction =      () -> new SimTimedAction("switching the pixel", 1);

    // sample action
    // this one is used by our simulator instead of moving motors/servos etc.
    static class  SimTimedAction implements Action  {
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

}
