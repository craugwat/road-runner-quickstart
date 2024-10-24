package org.firstinspires.ftc.teamcode.Hardware;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Base {
    private HardwareMap mhwMap = null;
    private DcMotorEx  fr   = null;
    private DcMotorEx  fl   = null;
    private DcMotorEx  br   = null;
    private DcMotorEx  bl   = null;
    public IMU imu  = null;


    private FtcDashboard dashboard = null;


    public Base(HardwareMap hwMap, FtcDashboard db) {
        RobotLog.d("8620WGW base constructor");
        mhwMap = hwMap;
        fr  = hwMap.get(DcMotorEx.class, "FR");
        fl  = hwMap.get(DcMotorEx.class, "FL");
        br  = hwMap.get(DcMotorEx.class, "BR");
        bl  = hwMap.get(DcMotorEx.class, "BL");
        dashboard = db;
    }

    public void init() {
        RobotLog.d("8620WGW base init");
        initMotor(fr, DcMotorSimple.Direction.FORWARD);
        initMotor(fl, DcMotorSimple.Direction.REVERSE);
        initMotor(br, DcMotorSimple.Direction.FORWARD);
        initMotor(bl, DcMotorSimple.Direction.REVERSE);
//        imu.resetHeading(0);

    }

    private void initMotor(DcMotor mtr, DcMotorSimple.Direction dir) {
        if (mtr != null) {
            mtr.setPower(0);
            mtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtr.setDirection(dir);
            mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            // need some rror handling here!
        }
    }




    double maxVel = 0.5;
    public void loop(Gamepad pad) {
        double frontLeft, backLeft, frontRight, backRight;  // temporary values
        double x_prime, y_prime;

        double x_axis =  expJoystick(-pad.left_stick_x,  maxVel);
        double y_axis = expJoystick(-pad.left_stick_y, maxVel);
        double rotate = expJoystick(pad.right_stick_x, maxVel);
        double theta    = 0;// Math.toRadians(-imu.getHeading());  // get direction is rboot facing

        //  Find robot's current axes in relation to original axes
        x_prime = x_axis * Math.cos(theta) + y_axis * Math.sin(theta);
        y_prime = -x_axis * Math.sin(theta) + y_axis * Math.cos(theta);

        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        //           fwd/back       rotate            straffe
        frontRight = y_prime - rotate + x_prime;
        backRight =  y_prime - rotate - x_prime;
        frontLeft =  y_prime + rotate - x_prime;
        backLeft =   y_prime + rotate + x_prime;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        // now send the speeds to the motors
        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);

        if (pad.y) {
//            imu.resetHeading(0);  // set our zero heading to the current robot heading
        }

        if (pad.left_stick_button)
            maxVel = 1.0;
        if (pad.right_stick_button)
            maxVel = 0.5;


//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Fl encoder", fl.getCurrentPosition());
//        packet.put("Fr encoder", fr.getCurrentPosition());
//        packet.put("bl encoder", bl.getCurrentPosition());
//        packet.put("br encoder", br.getCurrentPosition());
//        dashboard.sendTelemetryPacket(packet);
        RobotLog.d("8620WGW base  " +
                " fl " + fl.getPower() + ", " + fl.getCurrent(CurrentUnit.AMPS) +
                " fr " + fr.getPower() + ", " + fr.getCurrent(CurrentUnit.AMPS) +
                " bl " + bl.getPower() + ", " + bl.getCurrent(CurrentUnit.AMPS) +
                " br " + br.getPower() + ", " + br.getCurrent(CurrentUnit.AMPS) );
    }


    // map the joystick to an exponential curve to make driving easier
    // inputs - X is the joystick position,  max is the maximium output to motors 1 for full speed 0.5 for half speed etc.
    // notes:   signum() is just a way to get the sign of x.  It is +1 or -1
    // (max - deadBand) provides the range of the exponential portion
    public double expJoystick (double x, double max) {
        double deadBand = 0.1;  // how much of joystick movement around center should be ignored.
        double min = 0.1;       // minimum output to motors, this should just barely get motors moving.
        double out;         // Output to

        if (Math.abs(x) > deadBand) {
            double range = max - deadBand;
            out = ( (x * x) * range + min)  *  Math.signum(x);
        } else {
            out = 0;  // if joystick only moved slightly from center don't move!
        }
        return out;
    }

}
