/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is CoachBot
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class HardwareCoachbot
{
    public enum robotStates {
        idle,
        intake,
        lifting,
        armed,
        shooting,
        lowering
    }
    /* Public OpMode members. */
    HardwareMap hardwareMap =  null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public MecanumDrive drive = null;;
    Magazine magazine = null;
    Shooter shooter = null;
    Intake intake = null;
    Lift lift = null;

    /* local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareCoachbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        magazine = new Magazine(hardwareMap);
        magazine.init();

        shooter = new Shooter(hardwareMap);
        shooter.init();

        intake = new Intake(hardwareMap);
        intake.init();

        lift = new Lift(hardwareMap, dashboard);
        lift.init();
        lift.home();
    }


    robotStates state = robotStates.idle;
    robotStates lastState = robotStates.intake;

    // tele version of loop that uses joystick commands.
    public void loop(Gamepad myPad) {
        loop(myPad.left_bumper,  myPad.right_bumper,  (myPad.right_trigger > 0.5), (myPad.left_trigger > 0.5));
    }

    // generic verison of the loop can be used in tele or auto
    public void loop (boolean turnOn, boolean turnOff, boolean shoot1, boolean shoot3) {


        // the magazine and lift loops must run for this higher level loop to work!
        magazine.loop();
        lift.loop();


        // should we pause the state machine?
        if (turnOff && state != robotStates.idle) {
            lastState = state;
            state = robotStates.idle;
        }

        // should we turn on the state machine?
        if (turnOn && (state == robotStates.idle)) {
            state = lastState;
        }



        switch (state) {
            case idle:
                intake.off();
                shooter.shooterOff();
                break;
            case intake:
                intake.fullSpeed();
                if (magazine.getRingCntDebounce() == 3) {
                    intake.off();
                    lift.send2Top();        //.lift2Top();
                    state = robotStates.lifting;
                }
                break;

            case lifting:
                shooter.shooterFullSpeed();
                if(lift.isIdle())   // when lift motor finishes we are ready to shoot!
                    state = robotStates.armed;
                break;

            case armed: // read joystick here to start shooting
                shooter.shooterFullSpeed();
                if (shoot3) {
                    state = robotStates.shooting;
                    magazine.shootAll();
                } else if (shoot1) {
                    state = robotStates.shooting;
                    magazine.shootOne();
                }
                break;

            case shooting:
                shooter.shooterFullSpeed();
                if (magazine.isDoneShooting()) {
                    lift.send2Bottom();        //lift.lift2Bottom();
                    shooter.shooterOff();
                    state = robotStates.lowering;
                } else if (magazine.isIdle())
                    state = robotStates.armed;  // we shot one now ready to shoot again.
                break;

            case lowering:
                if(lift.isIdle())   // when lift motor finishes we are ready to shoot!
                    state = robotStates.intake;
                break;
        }
    }

    public String getLiftState () {
        return lift.getState().toString();
    }

    /**
     * expDrive() sets up exponential driving for teleOp.
     */
    double joyDead = 0.05;         // joystick range in which movement is considered accidental
    double motorMin = 0.05;         // minimum drive motor power

    public double expDrive(double joyVal, double maxVel) {
        if (Math.abs(joyVal) > joyDead) {
            double mOut = ((joyVal * joyVal) * (maxVel - motorMin)) / (1.0 - joyDead);
            double finalOutput = Math.copySign(mOut, joyVal);
            return finalOutput;
        }
        return 0;
    }

    public double getHeading() {
        double rawHeading = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading = rawHeading - offset;
        while (heading < 0)
            heading = heading + (2 * Math.PI);
        while (heading > (2 * Math.PI))
            heading = heading - (2 * Math.PI);
        return heading;
    }

    double offset = 0;
    public double resetHeading(double heading) {
        offset = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + heading;

        return offset;
    }

}

