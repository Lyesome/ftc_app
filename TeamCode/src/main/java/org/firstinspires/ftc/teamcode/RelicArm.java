package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by maris on 2018-01-13.
 */

public class RelicArm {
    public DcMotor motorRelicArm = null;
    public Servo relicGrab = null;
    public Servo relicLift = null;
    public Servo relicLock = null;
    public double RelicArmPower = 0.5;

    int POS_START;
    int POS_MAX;
    double UNLOCK = 0.0;
    double LOCK = 0.1;
    double RELIC_LIFT_DOWN = 0.42;
    double RELIC_LIFT_UP = 0;
    double RELIC_GRAB_OPEN = 0.71;
    double RELIC_GRAB_CLOSE = 0.5;

    HardwareMap myHWMap;

    public void RelicArm(){

    }
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        motorRelicArm = myHWMap.dcMotor.get("motor_relic_arm");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorRelicArm.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicGrab = myHWMap.servo.get("servo_relic_capture");
        relicLift = myHWMap.servo.get("servo_relic_lift");
        relicLock = myHWMap.servo.get("servo_relic_lock");
        relicLift.setPosition(RELIC_LIFT_DOWN);
        relicLock.setPosition(UNLOCK);
        relicGrab.setPosition(RELIC_GRAB_OPEN);

        //Set preset positions for Glyph Lifter
        POS_START = motorRelicArm.getCurrentPosition();
        POS_MAX = POS_START + 500;
    }

    public void Lift(){
        relicLift.setPosition(RELIC_LIFT_UP);
    }
    public void Lower(){
        relicLift.setPosition(RELIC_LIFT_DOWN);
    }

    public void Lock(){
        relicLock.setPosition(LOCK);
    }
    public void Unlock(){
        relicLock.setPosition(UNLOCK);
    }

    public void Grab(){
        relicGrab.setPosition(RELIC_GRAB_CLOSE);
    }
    public void Release(){
        relicGrab.setPosition(RELIC_GRAB_OPEN);
    }

    public void ArmExtension(double stick){

        int gotoPosition = motorRelicArm.getCurrentPosition();
        motorRelicArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (stick > 0 && motorRelicArm.getCurrentPosition() < POS_MAX) {
            gotoPosition = gotoPosition + 1;
            motorRelicArm.setTargetPosition(gotoPosition);
            motorRelicArm.setPower(RelicArmPower);
        }

        if (stick < 0 && motorRelicArm.getCurrentPosition() > POS_START) {
            gotoPosition = gotoPosition - 1;
            motorRelicArm.setTargetPosition(gotoPosition);
            motorRelicArm.setPower(RelicArmPower);
        }
        //if (stick > 0 && motorRelicArm.getCurrentPosition() < PositionMax) {
          //  motorRelicArm.setPower(RelicArmPower);
        //} else {
          //  if (stick < 0 && motorRelicArm.getCurrentPosition() > PositionMin) {
            //    motorRelicArm.setPower(-RelicArmPower);
            //} else {
              //  motorRelicArm.setPower(0);
            //}
       // }

    }

}



