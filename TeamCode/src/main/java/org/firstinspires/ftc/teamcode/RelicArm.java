package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Lyesome on 2018-01-13.
 * This class controls the Relic Arm and Relic Grabber
 */

public class RelicArm {
    public DcMotor motorRelicArm = null;
    public Servo relicGrab = null;
    public Servo relicLift = null;
    //public Servo relicLock = null; //Testing shows that relicLock servo is not needed
    public double RelicArmPower = 0.5; //Set speed of relic arm extension

    int POS_MIN;
    int POS_MAX;
    //double UNLOCK = 0.0;
    //double LOCK = 0.1;
    boolean LOCKED = false;
    double RELIC_LIFT_DOWN =  0.585;
    double RELIC_LIFT_UP =    0.66;
    double RELIC_GRAB_OPEN =  0.05;
    double RELIC_GRAB_CLOSE = 0.45;

    HardwareMap myHWMap;

    public void RelicArm(){ //constructor
    }

    public void initMotor(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        //Relic Extending Arm
        motorRelicArm = myHWMap.dcMotor.get("motor_relic_arm");
        motorRelicArm.setDirection(DcMotor.Direction.FORWARD);
        motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set minimum and maximum extension of the Relic Arm
        POS_MIN = motorRelicArm.getCurrentPosition();
        POS_MAX = POS_MIN + 6000;
    }
    //This method initializes the hardware on the relic arm
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;

        //Relic Grabber
        relicGrab = myHWMap.servo.get("servo_relic_capture");
        relicGrab.setDirection(Servo.Direction.REVERSE);
        relicGrab.setPosition(RELIC_GRAB_OPEN);
        relicLift = myHWMap.servo.get("servo_relic_lift");
        relicLift.setDirection(Servo.Direction.REVERSE);
        relicLift.setPosition(RELIC_LIFT_DOWN);
        //relicLock = myHWMap.servo.get("servo_relic_lock");
        //relicLock.setPosition(UNLOCK);

        //Relic Extending Arm
        motorRelicArm = myHWMap.dcMotor.get("motor_relic_arm");
        motorRelicArm.setDirection(DcMotor.Direction.FORWARD);
        motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set minimum and maximum extension of the Relic Arm
        POS_MIN = motorRelicArm.getCurrentPosition();
        POS_MAX = POS_MIN + 6000;
    }

    public void initServos(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;
        relicGrab = myHWMap.servo.get("servo_relic_capture");
        relicGrab.setDirection(Servo.Direction.REVERSE);
        relicGrab.setPosition(RELIC_GRAB_CLOSE);
        relicLift = myHWMap.servo.get("servo_relic_lift");
        relicLift.setDirection(Servo.Direction.REVERSE);
        relicLift.setPosition(RELIC_LIFT_DOWN);
        //relicLock = myHWMap.servo.get("servo_relic_lock");
    }

    //These methods are used to lift and lower the relic after it's been grabbed
    public void Lift(){
        relicLift.setPosition(RELIC_LIFT_UP);
    }
    public void Lower(){
        relicLift.setPosition(RELIC_LIFT_DOWN);
    }

//    public void Lock(){ relicLock.setPosition(LOCK); }  //Unused method for relicLock
//    public void Unlock(){ relicLock.setPosition(UNLOCK); } //Unused method for relicLock

    //This method grabs the relic and locks the grabber closed
    public void Grab(){
        relicGrab.setPosition(RELIC_GRAB_CLOSE);
        LOCKED = true;
    }

    //This method unlocks the grabber and releases the relic
    public void Release(){
        relicGrab.setPosition(RELIC_GRAB_OPEN);
        LOCKED = false;
    }

    //This method is used to manually control the relic grabber
    public void GrabberControl(double trigger){
        if (trigger > 0) {
            relicGrab.setPosition(trigger * (RELIC_GRAB_CLOSE-RELIC_GRAB_OPEN) + RELIC_GRAB_OPEN);
        }
    }

    //This method manually controls the extension of the relic arm
    //The speed of the motor is constant
    public void ArmExtension(double stick){
        if (stick > 0 && motorRelicArm.getCurrentPosition() < POS_MAX) {
            motorRelicArm.setPower(RelicArmPower);
        } else {
            if (stick < 0 && motorRelicArm.getCurrentPosition() > POS_MIN) {
                motorRelicArm.setPower(-RelicArmPower);
            } else {
                motorRelicArm.setPower(0);
            }
        }

    }

}



