package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by maris on 2018-01-13.
 */

public class GlyphLifter {

    HardwareMap myHWMap;
    public DcMotor motorLift = null;
    public Servo grabberR = null;
    public Servo grabberL = null;
    double GRABBER_START = 0.0;
    double GRABBER_OPEN = 0.3;
    double GRABBER_RELEASE = 0.6;
    double GRABBER_CLOSE = 0.8;
    boolean GRAB_LOCKED = false;
    int POS_START;
    int POS_MAX;
    int POS_1;
    int POS_2;
    int POS_3;

    public void GlyphLifter() {

    }
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        grabberL = myHWMap.servo.get("servo_glyph_left");
        grabberR = myHWMap.servo.get("servo_glyph_right");
        grabberL.setDirection(Servo.Direction.REVERSE);
        grabberR.setDirection(Servo.Direction.FORWARD);
        grabberL.setPosition(GRABBER_START);
        grabberR.setPosition(GRABBER_START);

        motorLift = myHWMap.dcMotor.get("motor_glyph_lifter");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        POS_START = motorLift.getCurrentPosition();
        POS_MAX = POS_START + 1900;
        POS_1 = POS_START + 100;
        POS_2 = POS_START + 750;
        POS_3 = POS_START + 1500;

    }

    public void Capture(){
        grabberL.setPosition(GRABBER_CLOSE);
        grabberR.setPosition(GRABBER_CLOSE);
        GotoPresetPosition(POS_2);
        //motorLift.setPower(0.2);
        //try {
        //    Thread.sleep(2000);
        //} catch (InterruptedException e){
        //    Thread.currentThread().interrupt();
        //}
        //motorLift.setPower(0.0);
    }

    public void GotoPresetPosition(int gotoPosition){
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setTargetPosition(gotoPosition);
        motorLift.setPower(0.6);
    }

    public void Grab(){
        grabberL.setPosition(GRABBER_CLOSE);
        grabberR.setPosition(GRABBER_CLOSE);
        GRAB_LOCKED = true;
    }

    public void Release(){
        grabberL.setPosition(GRABBER_RELEASE);
        grabberR.setPosition(GRABBER_RELEASE);
        GRAB_LOCKED = false;
    }

}
