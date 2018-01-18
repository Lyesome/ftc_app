package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by maris on 2018-01-13.
 */

public class RelicArm {
    public DcMotor motorRelicArm = null;
    public Servo relicGrab = null;
    public Servo relicLift = null;
    //public Servo relicLock = null;
    public double RelicArmPower = 0.5;

    int POS_MIN;
    int POS_MAX;
    double UNLOCK = 0.0;
    double LOCK = 0.1;
    boolean LOCKED = false;
    double RELIC_LIFT_DOWN = 0.585;
    double RELIC_LIFT_UP = 0.64;
    double RELIC_GRAB_OPEN = 0.21;
    double RELIC_GRAB_CLOSE = 0.45;

    HardwareMap myHWMap;

    public void RelicArm(){

    }
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;

        relicGrab = myHWMap.servo.get("servo_relic_capture");
        relicLift = myHWMap.servo.get("servo_relic_lift");
        //relicLock = myHWMap.servo.get("servo_relic_lock");
        relicLift.setDirection(Servo.Direction.REVERSE);
        relicLift.setPosition(RELIC_LIFT_DOWN);
        relicGrab.setDirection(Servo.Direction.REVERSE);
        relicGrab.setPosition(RELIC_GRAB_OPEN);
        //relicLock.setPosition(UNLOCK);

        motorRelicArm = myHWMap.dcMotor.get("motor_relic_arm");
        motorRelicArm.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Set preset positions for Glyph Lifter
        POS_MIN = motorRelicArm.getCurrentPosition();
        POS_MAX = POS_MIN + 6575;
    }

    public void Lift(){
        relicLift.setPosition(RELIC_LIFT_UP);
    }
    public void Lower(){
        relicLift.setPosition(RELIC_LIFT_DOWN);
    }

//    public void Lock(){ relicLock.setPosition(LOCK); }
//    public void Unlock(){ relicLock.setPosition(UNLOCK); }

    public void Grab(){
        LOCKED = true;
        relicGrab.setPosition(RELIC_GRAB_CLOSE);
    }
    public void Release(){
        LOCKED = false;
        relicGrab.setPosition(RELIC_GRAB_OPEN);
    }

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



