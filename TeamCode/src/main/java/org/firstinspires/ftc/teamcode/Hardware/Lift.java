package org.firstinspires.ftc.teamcode.Hardware;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private HardwareMap mhwMap = null;
    private DcMotor liftMtr = null;
    private DigitalChannel liftBottom;  // Hardware Device Object
    int liftTop = 2800;
    private FtcDashboard dashboard = null;

    public enum states {
        needinit,
        idle,
        homingBottom,
        findingTop,
        send2bottom,
        send2top,
        moving,

    }


    public Lift(HardwareMap hwMap, FtcDashboard db) {
        mhwMap = hwMap;
        liftMtr = hwMap.get(DcMotor.class, "liftMtr");
        liftBottom = hwMap.get(DigitalChannel.class, "liftBottom");
        liftBottom.setMode(DigitalChannel.Mode.INPUT);   // set the digital channel to input.\
        dashboard = db;
    }

    public void init() {
        // set lift motor motor
        liftMtr.setPower(0);
        liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMtr.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // make it easy to move until it is done with init.
    }

    public void homeBlocking() {
        // run down until no movement seen or 5 seconds
        long startTime = System.currentTimeMillis();
        liftMtr.setPower(-0.2);
        int lastPos = liftMtr.getCurrentPosition();
        while ((System.currentTimeMillis() - startTime) < 500) ;  // loop here to get motor moving
        int newPos = liftMtr.getCurrentPosition();
        while (Math.abs(newPos - lastPos) > 10 && ((System.currentTimeMillis() - startTime) < 5000)) {
            long loopTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - loopTime) < 100) ; // delay the loop
            lastPos = newPos;
            newPos = liftMtr.getCurrentPosition();
        }
        liftMtr.setPower(0);

        // WAIT A SECOND TO LET EVERYTHING STOP MOVING
        startTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startTime) < 500) ;  // loop here to get motor moving

        // SAVE THE BOTTOM ENCODER POSITION
        liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // now run to top stop
        startTime = System.currentTimeMillis();
        liftMtr.setPower(0.8);
        lastPos = liftMtr.getCurrentPosition();
        while ((System.currentTimeMillis() - startTime) < 500) ;  // loop here to get motor moving
        newPos = liftMtr.getCurrentPosition();
        while (Math.abs(newPos - lastPos) > 10 && ((System.currentTimeMillis() - startTime) < 5000)) {
            long loopTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - loopTime) < 100) ; // delay the loop
            lastPos = newPos;
            newPos = liftMtr.getCurrentPosition();
            if (liftMtr.getCurrentPosition() > 2000)
                liftMtr.setPower(0.2);
        }
        liftMtr.setPower(0);

        // WAIT A SECOND TO LET EVERYTHING STOP MOVING
        startTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startTime) < 500) ;  // loop here to get motor moving

        liftTop = liftMtr.getCurrentPosition();
    }

    public void lift2TopBlocking() {
        liftMtr.setTargetPosition(liftTop - 100);
        liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMtr.setPower(1);
    }

    public void lift2BottomBlocking() {
        liftMtr.setTargetPosition(100);
        liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMtr.setPower(1);
    }


// above this line is BLOCKING functions.  They will not return until things have completed.
/**************************************************************************************************/
// below this line are NON blocking functions.  They return without waiting for motors to finish

    public boolean needs2Home()
    {
        return (state == states.needinit);
    }

    public void home() {
        loop(states.homingBottom);
    }

    public double getPosition() {
        return liftMtr.getCurrentPosition();
    }

    public int getTop() {return liftTop;};

    public boolean isSafePosition() {
        return (liftMtr.getCurrentPosition() > 2000);
    }

    public boolean isIdle() {
        return (state == states.idle);
    }

    public states getState() {
        return state;
    }

    public  void send2Bottom () {
        loop(states.send2bottom);
    }

    public  void send2Top () {
        loop(states.send2top);
    }

    public states loop () {
        return (looper(null));
    }

    private states loop (states newstate) {
        return (looper(newstate));
    }

    private states state = states.needinit;
    long startTime;  // used as a timer for moves to timeout if it takes to long
    long timer;
    int lastPos;
    private states looper (states reqState) {
        int newPos;
        long timeNow,elapsedHomeTime, motionTime;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("LiftTop = ", liftTop);

        switch (state) {
            default:
            case needinit:
                if(reqState == states.homingBottom) {
                    state = states.homingBottom;
                    startTime = System.currentTimeMillis();
                    lastPos = liftMtr.getCurrentPosition();
                    liftMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // going to start moving so use the brake mode
                }
                break;

            case idle:
                if(reqState == states.send2bottom  || reqState == states.send2top) {
                    state = reqState;
                }
                break;

            case homingBottom:
                liftMtr.setPower(-1);
                if (! liftBottom.getState()) { // at bottom sensor
                    liftMtr.setPower(0);
                    liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // SAVE THE BOTTOM ENCODER POSITION
                    liftMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (liftTop == 0) {  // do we know the top position?
                        startTime = System.currentTimeMillis();
                        state = states.findingTop;
                    } else {
                        state = states.idle;
                    }
                }
//
//                newPos = liftMtr.getCurrentPosition();
//                timeNow = System.currentTimeMillis();
//                elapsedHomeTime = timeNow - startTime;
//                if (elapsedHomeTime > 500) {  // Iniital do nothing delay to get motor moving
//                    motionTime = timeNow - timer;
//                    if (Math.abs((newPos - lastPos)) < 10 ) {  //  not moving?
//                        if (motionTime > 100) {
//                            liftMtr.setPower(0);
//                            // SAVE THE BOTTOM ENCODER POSITION
//                            liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                            liftMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                            startTime = System.currentTimeMillis();
//                            state = states.findingTop;
//                        }
//                    } else {  // still moving so reset timer
//                        lastPos = newPos;
//                        timer = System.currentTimeMillis();
//                    }
//                } else {
//                    timer = System.currentTimeMillis();  // reset the timer for stop motion.
//                }
                break;

            case findingTop:
                if (liftMtr.getCurrentPosition() < 2000)
                    liftMtr.setPower(0.8);
                else
                    liftMtr.setPower(0.2);

                newPos = liftMtr.getCurrentPosition();
                timeNow = System.currentTimeMillis();
                elapsedHomeTime = timeNow - startTime;
                if (elapsedHomeTime > 500) {  // Iniital do nothing delay to get motor moving
                    motionTime = timeNow - timer;
                    if (Math.abs((newPos - lastPos)) < 10 ) {  //  not moving?
                        if (motionTime > 100) {
                            liftMtr.setPower(0);
                            // SAVE THE top ENCODER POSITION
                            liftTop = liftMtr.getCurrentPosition();
                            state = states.send2bottom;
                        }
                    } else {  // still moving so reset timer
                        lastPos = newPos;
                        timer = System.currentTimeMillis();
                    }
                } else {
                    timer = System.currentTimeMillis();  // reset the timer for stop motion.
                }


                break;

            case send2bottom: // initiate the move to the bottom
                liftMtr.setTargetPosition(0);
                liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMtr.setPower(1);
                state = states.moving;
                break;

            case moving:
                if (! liftMtr.isBusy())
                    state = states.idle;
               break;

            case send2top:
                liftMtr.setTargetPosition(liftTop-100);
                liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMtr.setPower(1);
                state = states.moving;
                break;
        }

        return state;
    }


}
