package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private HardwareMap mhwMap = null;
    public DcMotor intake   = null;


    public Intake(HardwareMap hwMap) {
        mhwMap = hwMap;
        intake  = hwMap.get(DcMotor.class, "intake");
    }


    public void init() {
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void fullSpeed () {
        intake.setPower(1);
    }

    public void reverse () {
        intake.setPower(-1);
    }

    public void off () {
        intake.setPower(0);
    }




}
