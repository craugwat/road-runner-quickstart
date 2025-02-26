package com.example.testrunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import javax.swing.JFrame;

public class RunOpModes extends JFrame {
    HardwareMap myMap = new HardwareMap(null, null);
    LinearOpMode opMode = new SplineTest();
    void runIt () throws InterruptedException {
        opMode.hardwareMap = myMap;

        opMode.runOpMode();
    }
}