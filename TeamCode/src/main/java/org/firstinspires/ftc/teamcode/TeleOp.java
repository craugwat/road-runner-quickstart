package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareCoachbot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp") 
public class TeleOp extends OpMode {
    HardwareCoachbot robot;

    @Override
    public void init() {
        robot   = new HardwareCoachbot();
        robot.init(hardwareMap);
    }

    double maxVel = 0.7;
    @Override
    public void loop() {
        robot.loop(gamepad1);

        double thetaRadians     = robot.getHeading();  // get direction is robot facing
        double joyStick_x_axis = robot.expDrive(-gamepad1.left_stick_x, maxVel);
        double joyStick_y_axis = robot.expDrive(-gamepad1.left_stick_y, maxVel);
        double  joystick_turn  = robot.expDrive(gamepad1.right_stick_x, maxVel*2/3);
        double robotStrafe     =  joyStick_x_axis * Math.cos(thetaRadians) + -joyStick_y_axis * Math.sin(thetaRadians);
        double robotForward    =  joyStick_x_axis * Math.sin(thetaRadians) + joyStick_y_axis * Math.cos(thetaRadians);

        // calculate motor powers based on:
        //                  forward/back         turn           strafe
        double frontRight   = robotForward - joystick_turn + robotStrafe;
        double backRight    = robotForward - joystick_turn - robotStrafe;
        double frontLeft    = robotForward + joystick_turn - robotStrafe;
        double backLeft     = robotForward + joystick_turn + robotStrafe;

        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }
        // directly set motor powers rather than use setDrivePowers() from RR
        robot.drive.leftFront.setPower(frontLeft);
        robot.drive.leftBack.setPower(backLeft);
        robot.drive.rightFront.setPower(frontRight);
        robot.drive.rightBack.setPower(backRight);
        // regular/slow mode
        if (gamepad1.dpad_down)
            maxVel = 0.4;       // slow mode
        else if (gamepad1.dpad_right)
            maxVel = 0.7;        // speed mode
        if (gamepad1.y)
            robot.resetHeading(0);


        telemetry.addLine("Loop State = "+ robot.getLiftState());
        telemetry.update();
    }
}
