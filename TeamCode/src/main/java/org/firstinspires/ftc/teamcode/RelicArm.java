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
    public double RelicArmPower;

    double PositionMin;
    double PositionMax;

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
        RelicArmPower = 0.5;
        double UNLOCK = 0.0;
        double LOCK = 0.1;
        double RELIC_LIFT_DOWN = 0.42;
        double RELIC_LIFT_UP = 0;
        double RELIC_GRAB_OPEN = 0.71;
        double RELIC_GRAB_CLOSE = 0.5;
        relicLift.setPosition(RELIC_LIFT_DOWN);
        relicLock.setPosition(UNLOCK);
        relicGrab.setPosition(RELIC_GRAB_OPEN);

        //Set preset positions for Glyph Lifter
        PositionMin = motorRelicArm.getCurrentPosition();
        PositionMax = PositionMin + 500;
    }

    public void Lift(){
        relicLift.setPosition(0.5);
    }
    public void Lower(){
        relicLift.setPosition(0.2);
    }

    public void Lock(){
        relicLock.setPosition(0.5);
    }
    public void Unlock(){
        relicLock.setPosition(0.0);
    }

    public void Grab(){
        relicGrab.setPosition(0.5);
    }
    public void Release(){
        relicGrab.setPosition(0.0);
    }

    public void ArmExtension(double stick){
        if (stick > 0 && motorRelicArm.getCurrentPosition() < PositionMax) {
            motorRelicArm.setPower(RelicArmPower);
        } else {
            if (stick < 0 && motorRelicArm.getCurrentPosition() > PositionMin) {
                motorRelicArm.setPower(-RelicArmPower);
            } else {
                motorRelicArm.setPower(0);
            }
        }

    }

}



