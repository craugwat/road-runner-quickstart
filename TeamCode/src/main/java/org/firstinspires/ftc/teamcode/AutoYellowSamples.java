package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.trajectoryactions.SampleOpModes.YellowSamples;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public final class AutoYellowSamples extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        YellowSamples as = new YellowSamples(drive);

        waitForStart();
        Actions.runBlocking(as.allYellows());
    }

}
