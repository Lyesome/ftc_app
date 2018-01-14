package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by maris on 2018-01-13.
 */

public class GlyphLifter {

    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap myHWMap;
    private DcMotor motorLift = null;
    private Servo GrabberR = null;
    private Servo GrabberL = null;


    public void GlyphLifter() {

    }
    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        motorLift = myHWMap.dcMotor.get("motor_glyph_lifter");
        GrabberL = myHWMap.servo.get("servo_glyph_left");
        GrabberR = myHWMap.servo.get("servo_glyph_right");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        GrabberL.setPosition(0.5);
        GrabberR.setPosition(0.5);
    }

    public void Capture(){
        GrabberL.setPosition(0.2);
        GrabberR.setPosition(0.8);
        motorLift.setPower(0.2);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
        motorLift.setPower(0.0);
    }
    public void Release(){
        GrabberL.setPosition(0.5);
        GrabberR.setPosition(0.5);

    }
    public void GlyphLIft(){}

}
