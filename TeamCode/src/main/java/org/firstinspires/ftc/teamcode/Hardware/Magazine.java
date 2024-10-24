package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Magazine {
    private HardwareMap mhwMap = null;
    private Servo pusherServo    = null;
    private DigitalChannel ramSensor;  // Hardware Device Object
    private DistanceSensor ringSensor;

    private double   ramStopSpeed = 0.5;
    private double   ramFWDSpeed = 1.0;
    private double   ramHomepeed = 1.0;

    enum states {
        idle,
        manualHome,
        home,
        shootOne,
        shootAll
    }

    public Magazine(HardwareMap hwMap) {
        mhwMap = hwMap;
        pusherServo  = hwMap.get(Servo.class, "pusherServo");

        ramSensor = hwMap.get(DigitalChannel.class, "ramSensor");
        ramSensor.setMode(DigitalChannel.Mode.INPUT);   // set the digital channel to input.\

        ringSensor = mhwMap.get(DistanceSensor .class, "ringSensor");

    }

    public void init() {
        pusherServo.setPosition(0.5);

    }

    public void pusherOn () {
        pusherServo.setPosition(ramFWDSpeed);
    }

    public void pusherOff () {
        pusherServo.setPosition(ramStopSpeed);
    }

    public void pusherHome () {
        pusherServo.setPosition(ramHomepeed);
        while ( ramSensor.getState()) ;     // loop here until sensor activated - should add a timeout?
        pusherServo.setPosition(ramStopSpeed);
    }

    public boolean isPusherHome() {
        return ( ! ramSensor.getState());
    }

    // returns ring coutn based on sensor only, no debounce!
    public int getRingCnt() {
        double dist = ringSensor.getDistance(DistanceUnit.MM);
        int rings = 0;
        if (dist < 110) rings = 1;
        if (dist < 85) rings = 2;
        if (dist < 60) rings = 3;

        return (rings);
    }

    public int emptyAllRings() {
        pusherOn();
        while (getRingCnt() > 0);  // loop here until empty - should add a time out?
        pusherHome();
        return (getRingCnt());
    }

    public int emptyOneRing() {
        int StartRings = getRingCnt();

        if (getRingCnt() > 0) {
            pusherOn();
            while (getRingCnt() >= StartRings) ;  // loop here until we shoot 1, could also watch ram sensor?
            pusherHome();
        }
        return (getRingCnt());
    }

// above this line is BLOCKING functions.  They will not return until things have completed.
    /**************************************************************************************************/
// below this line are NON blocking functions.  They return without waiting for motors to finish


    // non blocking methods to run the magazine
    private long startTime = System.currentTimeMillis();
    private states state = states.manualHome;
    private int lastRingCnt = 0;
    private int currentRingCnt = 0;
    private int startRing = 0;

    // return a debounced version of ring count.  this is not sensitive to brief blocking of sensor
    // such as when a new ring is thrown in to the magazine by the intake.
    // note loop() must be called for this to update!
    public int getRingCntDebounce() {
        return(currentRingCnt);
    }

    public boolean needs2Home () {
        return (state == states.manualHome);
    }

    public boolean isIdle() {
        return (state == states.idle);
    }

    public boolean isDoneShooting () {
        return (state == states.idle && getRingCntDebounce() == 0);
    }

    // need some way to know when to init the ram
    // user can put hand in front of ring sensor to beging init
    public boolean safe2Init() {
        return (ringSensor.getDistance(DistanceUnit.MM) < 20);
    }

    public void homeRam() {
        loop (states.home);
    }

    public void shootAll() {
        looper(states.shootAll);
    }

    public void shootOne() {
        looper(states.shootOne);
    }

    public states loop () {
        return (looper(null));
    }

    private states loop (states newstate) {
        return (looper(newstate));
    }


    states looper (states reqState) {
        // first debounce the ring counter sensor. Incoming rings can cause false reading of sensor.
        int ringsNow = getRingCnt();
        if (lastRingCnt == ringsNow) {
            if ((System.currentTimeMillis() - startTime) > 250)  // must be same count for this many milliseconds
                currentRingCnt = ringsNow;
        } else {
            startTime = System.currentTimeMillis();  // different value so resent the time counter
            lastRingCnt = ringsNow;
        }

        if (reqState == states.home)
            state = reqState;

        switch (state) {
            case idle:
                if (reqState == states.shootAll) {
                    state = states.shootAll;
                }
                if (reqState == states.shootOne) {
                    state = states.shootOne;
                    startRing = currentRingCnt;
                }
                break;

            case manualHome:  // let user put us in a safe position at our sensor before we move!
                if  ( ! ramSensor.getState()) {
                    state = states.idle;
                }
                break;

           case shootOne:
               pusherOn();
               if (currentRingCnt < startRing  ||  currentRingCnt == 0) {  // did it go down 1 ring?
                   state = states.home;
               }
                break;

            case shootAll:
                pusherOn();
                if (currentRingCnt == 0) {  // did we shoot them all?
                    state = states.home;
                }
                break;

            case home:
                pusherOn();
                if ( ! ramSensor.getState()) {
                    pusherOff();
                    state = states.idle;
                }
                break;

            default:
                pusherOff();
                state = states.idle;
                break;

        }

        return state;
    }

    public states getState() {
        return state;
    }
}
