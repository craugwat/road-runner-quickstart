package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

public class TrajSplineTest {
    public TrajSplineTest(TrajectoryActionBuilder trajActBld) {
        trajActBld.splineTo(new Vector2d(30, 36), 0);
        trajActBld.splineTo(new Vector2d(60, 0), Math.PI * 3/2);
    }
}
