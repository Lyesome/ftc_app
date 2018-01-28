package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Lyesome on 2018-01-13.
 * This class contains all the methods for controlling the Glyph Lifter
 */

public class GlyphLifter {

    HardwareMap myHWMap;
    public      DcMotor motorLift   = null;
    public      Servo grabberR      = null;
    public      Servo grabberL      = null;
    double      GRABBER_START       =  0.0;
    double      GRABBER_OPEN        =  0.3;
    double      GRABBER_RELEASE     =  0.5;
    double      GRABBER_CLOSE       =  0.7;
    double      LIFT_UP_POWER       =  0.5;
    double      LIFT_DOWN_POWER     =  0.3;
    boolean     GRAB_LOCKED         = false;
    int         POS_START;
    int         POS_MAX;
    int         POS_1;
    int         POS_2;
    int         POS_3;

    public void GlyphLifter() { //constructor
    }

    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;

        //Grabber Initialization
        grabberL = myHWMap.servo.get("servo_glyph_left");
        grabberR = myHWMap.servo.get("servo_glyph_right");
        grabberL.setDirection(Servo.Direction.REVERSE);
        grabberR.setDirection(Servo.Direction.FORWARD);

        grabberL.setPosition(GRABBER_START);
        grabberR.setPosition(GRABBER_START);

        //Lifter Initialization
        motorLift = myHWMap.dcMotor.get("motor_glyph_lifter");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        POS_START = motorLift.getCurrentPosition();
        POS_MAX   = POS_START + 1900;
        POS_1     = POS_START + 100;
        POS_2     = POS_START + 750;
        POS_3     = POS_START + 1500;
    }

    //Method for manual control of the Glyph Grabbers
    public void GrabberControl(double trigger){
        if (trigger > 0) {
            grabberR.setPosition(trigger * (GRABBER_CLOSE - GRABBER_OPEN) + GRABBER_OPEN);
            grabberL.setPosition(trigger * (GRABBER_CLOSE - GRABBER_OPEN) + GRABBER_OPEN);
        }
    }

    //Method for manual control of the Lifter
    public void LifterControl(double stick){
        //Make sure Lifter motor is using encoder when going to preset positions
        //Do NOT reset the encoder or min and max positions will change
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if ((stick > 0) && (motorLift.getCurrentPosition() <= POS_MAX)) {
            //POS_MAX limit is to prevent operator from trying to raise Lifter too high
            motorLift.setPower(stick*LIFT_UP_POWER);
        }else {
            if ((stick < 0) && (motorLift.getCurrentPosition() >= POS_1)) {
                //POS_1 limit is to prevent operator from hitting the ground
                motorLift.setPower(stick*LIFT_DOWN_POWER);
            } else {
                motorLift.setPower(0);
            }
        }
    }

    //Method to automatically grab Gylph and lift it high enough off ground so it doesn't get in the way of driving
    public void Capture(){
        grabberL.setPosition(GRABBER_CLOSE);
        grabberR.setPosition(GRABBER_CLOSE);
        GotoPresetPosition(POS_2);
    }

    //Method to automatically move Lifter to specified position
    //Position is specified as encoder values, not inches
    public void GotoPresetPosition(int gotoPosition){
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setTargetPosition(gotoPosition);
        motorLift.setPower(0.6);
    }

    //Method to automatically grab Gylph
    public void Grab(){
        grabberL.setPosition(GRABBER_CLOSE);
        grabberR.setPosition(GRABBER_CLOSE);
        GRAB_LOCKED = true; //Set GRAB_LOCKED state for external inquiry
    }

    //Method to automatically release Gylph
    public void Release(){
        grabberL.setPosition(GRABBER_RELEASE);
        grabberR.setPosition(GRABBER_RELEASE);
        GRAB_LOCKED = false; //Set GRAB_LOCKED state for external inquiry
    }

}
