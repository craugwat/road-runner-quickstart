package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuning.SplineTest;
import javax.swing.*;


public class TestOpModeRunner {


    HardwareMap myMap = new HardwareMap(null, null);
    LinearOpMode opMode = new SplineTest();
    void runIt () throws InterruptedException {
        opMode.hardwareMap = myMap;

        opMode.runOpMode();
    }
}
