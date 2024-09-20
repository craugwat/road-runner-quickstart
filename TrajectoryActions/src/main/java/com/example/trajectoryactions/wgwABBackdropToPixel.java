package com.example.trajectoryactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;

public class wgwABBackdropToPixel {
    public Action backdropToWhitePixelTruss(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToSplineHeading(new Pose2d(24,60 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
                .build());
    }
}
