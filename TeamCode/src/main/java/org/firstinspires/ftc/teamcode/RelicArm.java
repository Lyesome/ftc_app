package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by maris on 2018-01-13.
 */

public class RelicArm {
    public DcMotor motorRelicArm = null;
    public Servo RelicGrab = null;
    public Servo RelicLift = null;
    public Servo RelicLock = null;
    public double RelicArmPower;

    double PositionMin;
    double PositionMax;

    HardwareMap myHWMap;

    public void RelicArm(){

    }
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        motorRelicArm = myHWMap.dcMotor.get("motor_ra");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorRelicArm.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RelicGrab = myHWMap.servo.get("servo_relic_grab");
        RelicLift = myHWMap.servo.get("servo_relic_lift");
        RelicLock = myHWMap.servo.get("servo_relic_lock");
        RelicArmPower = 0.5;
        //Set preset positions for Glyph Lifter
        PositionMin = motorRelicArm.getCurrentPosition();
        PositionMax = PositionMin + 500;
    }

    public void Lift(){
        RelicLift.setPosition(0.5);
    }
    public void Lower(){
        RelicLift.setPosition(0.2);
    }

    public void Lock(){
        RelicLock.setPosition(0.5);
    }
    public void Unlock(){
        RelicLock.setPosition(0.0);
    }

    public void Grab(){
        RelicGrab.setPosition(0.5);
    }
    public void Release(){
        RelicGrab.setPosition(0.0);
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



