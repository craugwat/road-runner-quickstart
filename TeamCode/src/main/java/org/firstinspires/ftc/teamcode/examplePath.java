package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public final class examplePath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            TrajectoryActionBuilder trajActBld = drive.actionBuilder(drive.pose);
            trajActBld.splineTo(new Vector2d(30, 36), 0);
            trajActBld.splineTo(new Vector2d(60, 0), Math.PI * 3/2);
            Actions.runBlocking(trajActBld.build());

//            Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(30, 36), 0)
//                        .splineTo(new Vector2d(60, 0), Math.PI * 3/2)
//                        .build());
    }
}
