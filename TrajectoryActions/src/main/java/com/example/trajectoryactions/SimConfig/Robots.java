package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.example.trajectoryactions.Samples.AutoSpecimens;
import com.example.trajectoryactions.Samples.AutoYellowSamples;
import com.example.trajectoryactions.Samples.commonTrajectories;
import com.example.trajectoryactions.frontDoorDriving;
import com.example.trajectoryactions.frontTrussDriving;
import com.example.trajectoryactions.wgwABCommon;
import com.example.trajectoryactions.wgwABFarSideDriving;
import com.example.trajectoryactions.wgwABNearSideDriving;

import java.util.ArrayList;
import java.util.function.Supplier;

public class Robots {

    //    public LinkedHashMap<String, SimRobot> simRobots = new LinkedHashMap<>();
    public ArrayList<SimRobot> simRobots = new ArrayList<>();

    public enum FieldBackground {
        centerStage,
        intoTheDeep
    }
    public FieldBackground background = FieldBackground.intoTheDeep;  // define this variable to set the background


    public Robots() {
        // add puts robot into menu
        // boolean past to constructor
        //   true=enable the robot to run by default
        //   flase= don't run in simulaotr, user must enable in menu
        simRobots.add(new RedSideAllYellows(true));
        simRobots.add(new BlueSideAllYellows(true));

        simRobots.add(new RedAutoSpecimens(true));
        simRobots.add(new BlueAutoSpecimens(true));


    }

    public void startGameTimer() {
//        wgwABCommon.startGameTimer();
    }

    private class RedSideAllYellows extends SimRobot {

        RedSideAllYellows(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Red Side All Yellows";
            AutoYellowSamples autoYellow = new AutoYellowSamples(drive);
            autoYellow.actionParameters.fieldSide = commonTrajectories.FieldSide.RED;
            paths.put("Sample Yellow", () -> autoYellow.allYellows(drive));
        }
    }

    private class BlueSideAllYellows extends SimRobot {

        BlueSideAllYellows(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Blue Side All Yellows";
            AutoYellowSamples autoYellow = new AutoYellowSamples(drive);
            autoYellow.actionParameters.fieldSide = commonTrajectories.FieldSide.BLUE;
            paths.put("Sample Yellow", () -> autoYellow.allYellows(drive));
        }
    }

    private class RedAutoSpecimens extends SimRobot {

        RedAutoSpecimens(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Red Specimens";
            AutoSpecimens autoSpecimens = new AutoSpecimens(drive);
            autoSpecimens.actionParameters.fieldSide = commonTrajectories.FieldSide.RED;
            paths.put("Sample Yellow", () -> autoSpecimens.allSpecimens(drive));
        }
    }

    private class BlueAutoSpecimens extends SimRobot {

        BlueAutoSpecimens(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Blue Specimens";
            AutoSpecimens autoSpecimens = new AutoSpecimens(drive);
            autoSpecimens.actionParameters.fieldSide = commonTrajectories.FieldSide.BLUE;
            paths.put("Sample Yellow", () -> autoSpecimens.allSpecimens(drive));
        }
    }
}