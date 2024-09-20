package com.example.trajectoryactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;

public class backdropToPixelStackDriving {

    // this action will drive from backdrop to white pixel stack under the stage door
    public static Action backdropToPixelStackDoor(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToSplineHeading(new Pose2d(24, 12 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .splineToSplineHeading(new Pose2d(-58, 12 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .build());
    }

    public static Action backdropToPixelStackDoor1(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToSplineHeading(new Pose2d(36, 24 * dir, Math.toRadians(270 * dir)), Math.toRadians(270 * dir))
                .splineToSplineHeading(new Pose2d(24, 12 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .build());
    }

    public static Action backdropToPixelStackDoor2(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToSplineHeading(new Pose2d(-58, 12 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .build());
    }

    public static Action pixelStackToBackstageDoor(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24,12 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(48,12 * dir, Math.toRadians(0 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    public static Action backdropToPixelStackTrusses1(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToSplineHeading(new Pose2d(36, 48 * dir, Math.toRadians(90 * dir)), Math.toRadians(90 * dir))
                .splineToSplineHeading(new Pose2d(24, 60 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .build());
    }

    public static Action backdropToPixelStackTrusses2(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToSplineHeading(new Pose2d(-46, 60 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir))
                .splineToConstantHeading(new Vector2d(-57, 36 * dir), Math.toRadians(270 * dir))
                .build());
    }

    public static Action pixelStackToBackstageTrusses(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == wgwABCommon.FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-46,60 * dir), Math.toRadians(0 * dir))
//                .splineToConstantHeading(new Vector2d(36,60 * dir), Math.toRadians(0 * dir))
                .splineToLinearHeading(new Pose2d(36,60 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
                .splineToSplineHeading(new Pose2d(46, 58 * dir, Math.toRadians(358 * dir)), Math.toRadians(0 * dir))
                .build());
    }

}


