package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

//        simRobots.add(new MM_B_C(true));
//        simRobots.add(new MM_B_FD(true));
//        simRobots.add(new MM_R_C(true));
//        simRobots.add(new MM_R_FD(true));
//
//        simRobots.add(new MM_B_F(false));
//        simRobots.add(new MM_B_FT(false));
//        simRobots.add(new MM_R_F(false));
//        simRobots.add(new MM_R_FT(false));

    }

    public void startGameTimer() {
//        wgwABCommon.startGameTimer();
    }

    private Supplier<Action> farSimBuilder(Drive drive, wgwABCommon.FieldSide side, wgwABCommon.SelectedSpike spike) {
        simActions sa = new simActions();
        wgwABFarSideDriving abFarSideDriving = new wgwABFarSideDriving(drive);
//        wgwABFarSideDriving.actionParameters aP = new abFarSideDriving.actionParameters();
        return () -> abFarSideDriving.far(drive, sa.dropPurple.get(), sa.dropYellow.get(), sa.stateMachineAction.get(), sa.stateMachineAction.get(), side, spike, sa.tagAction.get(), sa.waitForRobotAction.get(), sa.clawSwitchAction.get());
    }

    ;

    private Supplier<Action> closeSimBuilder(Drive drive, wgwABCommon.FieldSide side, wgwABCommon.SelectedSpike spike, wgwABCommon.ParkingPosState parkingPos) {
        wgwABNearSideDriving abNearSideDriving = new wgwABNearSideDriving(drive);
        simActions sa = new simActions();
        return () -> abNearSideDriving.close(drive, sa.dropPurple.get(), sa.dropYellow.get(), sa.stateMachineAction.get(), sa.stateMachineAction.get(), side, spike, sa.tagAction.get(), parkingPos);
    }

    ;

    private Supplier<Action> farFrontDoorSimBuilder(Drive drive, wgwABCommon.FieldSide side, wgwABCommon.SelectedSpike spike) {
        simActions sa = new simActions();
        frontDoorDriving frontDoorDriving = new frontDoorDriving(drive);
        return () -> frontDoorDriving.farDoor(drive, sa.dropPurple.get(), sa.dropYellow.get(), sa.stateMachineAction.get(), sa.stateMachineAction.get(), side, spike, sa.tagAction.get(), sa.waitForRobotAction.get(), sa.clawSwitchAction.get());
    }

    ;

    private Supplier<Action> farTrussSimBuilder(Drive drive, wgwABCommon.FieldSide side, wgwABCommon.SelectedSpike spike) {
        simActions sa = new simActions();
        frontTrussDriving frontTrussDriving = new frontTrussDriving(drive);
        return () -> frontTrussDriving.farTrusses(drive, sa.dropPurple.get(), sa.dropYellow.get(), sa.stateMachineAction.get(), sa.stateMachineAction.get(), side, spike, sa.tagAction.get(), sa.waitForRobotAction.get(), sa.clawSwitchAction.get());
    }

    ;


    private class MM_B_F extends SimRobot {

        MM_B_F(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_B_F";
            paths.put("Blue Far Side Left", farSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.LEFT));
            paths.put("Blue Far Side Center", farSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.MIDDLE));
            paths.put("Blue Far Side Right", farSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.RIGHT));
        }
    }

    private class MM_B_C extends SimRobot {

        MM_B_C(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_B_C";
            paths.put("Blue Close Side Left", closeSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.LEFT, wgwABCommon.ParkingPosState.CORNER));
            paths.put("Blue Close Side Center", closeSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.MIDDLE, wgwABCommon.ParkingPosState.CORNER));
            paths.put("Blue Close Side Right", closeSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.RIGHT, wgwABCommon.ParkingPosState.CORNER));
        }
    }

    private class MM_B_FD extends SimRobot {

        MM_B_FD(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_B_Front Door";
            paths.put("Blue Close Side Left", farFrontDoorSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.LEFT));
            paths.put("Blue Close Side Center", farFrontDoorSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.MIDDLE));
            paths.put("Blue Close Side Right", farFrontDoorSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.RIGHT));
        }
    }

    private class MM_B_FT extends SimRobot {

        MM_B_FT(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_B_Front Truss";
            paths.put("Blue Close Side Left", farTrussSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.LEFT));
            paths.put("Blue Close Side Center", farTrussSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.MIDDLE));
            paths.put("Blue Close Side Right", farTrussSimBuilder(drive, wgwABCommon.FieldSide.BLUE, wgwABCommon.SelectedSpike.RIGHT));
        }
    }


    private class MM_R_C extends SimRobot {

        MM_R_C(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_R_C";

            /** BLUE CLOSE */
            paths.put("Blue Close Side Left", closeSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.LEFT, wgwABCommon.ParkingPosState.CORNER));
            paths.put("Blue Close Side Center", closeSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.MIDDLE, wgwABCommon.ParkingPosState.CORNER));
            paths.put("Blue Close Side Right", closeSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.RIGHT, wgwABCommon.ParkingPosState.CORNER));
        }
    }

    private class MM_R_F extends SimRobot {

        MM_R_F(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_R_F";

            /** RED FAR */
            paths.put("Red Far Side Left", farSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.LEFT));
            paths.put("Red Far Side Center", farSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.MIDDLE));
            paths.put("Red Far Side Right", farSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.RIGHT));
        }
    }
    private class MM_R_FD extends SimRobot {

        MM_R_FD(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_R_Front Door";
            paths.put("Red Close Side Left", farFrontDoorSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.LEFT));
            paths.put("Red Close Side Center", farFrontDoorSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.MIDDLE));
            paths.put("Red Close Side Right", farFrontDoorSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.RIGHT));
        }
    }
    private class MM_R_FT extends SimRobot {

        MM_R_FT(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "MM_R_Front Truss";
            paths.put("Red Close Side Left", farTrussSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.LEFT));
            paths.put("Red Close Side Center", farTrussSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.MIDDLE));
            paths.put("Red Close Side Right", farTrussSimBuilder(drive, wgwABCommon.FieldSide.RED, wgwABCommon.SelectedSpike.RIGHT));
        }
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

}