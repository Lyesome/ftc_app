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
    double GRABBER_START = 0.3;
    double GRABBER_OPEN = 0.5;
    double GRABBER_CLOSE = 0.7;

    public void GlyphLifter() {

    }
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        motorLift = myHWMap.dcMotor.get("motor_glyph_lifter");
        grabberL = myHWMap.servo.get("servo_glyph_left");
        grabberR = myHWMap.servo.get("servo_glyph_right");
        grabberL.setDirection(Servo.Direction.FORWARD);
        grabberR.setDirection(Servo.Direction.REVERSE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        double POS_START = motorLift.getCurrentPosition();
        double POS_MAX = POS_START + 1680;
        double POS_1 = POS_START + 100;
        double POS_2 = POS_START + 750;
        double POS_3 = POS_START + 1350;

        grabberL.setPosition(GRABBER_START);
        grabberR.setPosition(GRABBER_START);

    }

    public void Capture(){
        grabberL.setPosition(GRABBER_CLOSE);
        grabberR.setPosition(GRABBER_CLOSE);
        motorLift.setPower(0.2);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
        motorLift.setPower(0.0);
    }
    public void Release(){
        grabberL.setPosition(GRABBER_OPEN);
        grabberR.setPosition(GRABBER_OPEN);

    }
    public void GotoPresetPosition(double gotoPosition){



    }

}
