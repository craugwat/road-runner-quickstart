package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Actions.now;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.trajectoryactions.SampleTrajectories.AutoSpecimens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 ---------------------------------------------------------------------------------
 this opmode demonstrates how to share trajectories with the BeepBeep Simulator.
 Trajectories are stored in a separate file that is a class accessible by the
 Simulator and robot code.
 Examples of Actions that move motors and servos are provided here.  However you might
 want to move these to a different class that is accessible by all your opmodes to
 reduce duplicating code.
 ---------------------------------------------------------------------------------
*/

@Config
@Autonomous
public final class AutoSampleBeepBeep extends LinearOpMode {

    DcMotor liftMtr;
    Servo clawServo;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        liftMtr = hardwareMap.get(DcMotor.class, "liftMtr");
        liftMtr.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawServo  = hardwareMap.get(Servo.class, "clawServo");


        AutoSpecimens autoSpecimens = new AutoSpecimens(drive);
        autoSpecimens.actionParameters.collectSample = moveClawServ(0.25, 1.0);
        autoSpecimens.actionParameters.deliverSample = moveClawServ(0.75, 1.0);
        autoSpecimens.actionParameters.liftUp = moveLift(2000, 5.0);
        autoSpecimens.actionParameters.liftDown = moveLift(100, 5.0);


        waitForStart();

//        Actions.runBlocking(autoSpecimens.actionTest());
        Actions.runBlocking(autoSpecimens.allSpecimens());
    }

    // This method returns and action that moves our claw servo.
    // Servo's do not provide feedback on position, so it commands the servo and then
    // waits for a timer to expire.
    // When the action is complete it resets the timer variables so it can be run again.
    public Action moveClawServ(double position, double timeout) {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = timeout;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){                        // first time to run
                    clawServo.setPosition(position);    // command servo to new position
                    beginTs = now();                    // record time we start running
                } else {
                    t = now()-beginTs;                  // how long have we been running
                }
                p.put("claw", clawServo.getPosition());
                p.put("Claw Timer", t);

                if (t<dt) {
                    return true;                       // actions are run until they return false;
                } else {
                    beginTs = -1;                       // reset so it can be run again
                    t = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template reequired it to.
        };
    }

    // This method returns and action that controls moving our lift up and down.
    // It runs until the motor to finish moving or a timer expires.
    // So it may return before motor reaches setpoint, depending on motor speed and timeout set.
    // When the action is complete it resets the timer variables so it can be run again.
    public Action moveLift(int position, double timeout) {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = timeout;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){                        // first time to run
                    liftMtr.setTargetPosition(position);
                    liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMtr.setPower(0.5);
                    beginTs = now();                    // record time we start running
                } else {
                    t = now()-beginTs;                  // how long have we been running
                }
                p.put("lift", liftMtr.getCurrentPosition());
                p.put("lift Timer", t);

                if (t<dt && liftMtr.isBusy()) {
                    return true;                       // actions are run until they return false;
                } else {
                    beginTs = -1;                       // reset so it can be run again
                    t = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template reequired it to.
        };
    }
}
