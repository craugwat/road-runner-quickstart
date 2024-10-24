package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private HardwareMap mhwMap = null;
    public DcMotor  leftShooter   = null;
    public DcMotor rightShooter   = null;


    public Shooter(HardwareMap hwMap) {
        mhwMap = hwMap;
        leftShooter  = hwMap.get(DcMotor.class, "leftShooter");
        rightShooter  = hwMap.get(DcMotor.class, "rightShooter");
    }

    public void init() {
        leftShooter.setPower(0);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooter.setPower(0);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void shooterFullSpeed () {
        leftShooter.setPower(0.5);
        rightShooter.setPower(0.5);
    }

    public void shooterOff () {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }



}
