package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Lyesome on 2018-01-13.
 * This class is used to pass the gamepad controls to generalized methods that the other classes can use.
 * This allows all the controls to be defined in one location and makes it easier to change the controls if need.
 */

public class BotControls {

    public BotControls()  { // constructor
    }

//GAMEPAD 1
    //Drive controls
    public static double  DriveYStick(LinearOpMode op){ return          op.gamepad1.left_stick_y; }
    public static double  DriveXStick(LinearOpMode op){ return          op.gamepad1.left_stick_x; }
    public static double  TurnStick(LinearOpMode op){
        return           op.gamepad1.right_stick_x;
    }
    public static double  DriveThrottle(LinearOpMode op){
        return       op.gamepad1.right_trigger;
    }

//GAMEPAD 2
    //Glyph Lifter controls
    public static double  GlyphGrabTrigger(LinearOpMode op){ return     op.gamepad2.right_trigger; }
    public static double  GlyphLiftStick(LinearOpMode op){ return       op.gamepad2.right_stick_y; }
    public static boolean GlyphLiftMinButton(LinearOpMode op){ return   op.gamepad2.a; }
    public static boolean GlyphLiftLowButton(LinearOpMode op){ return   op.gamepad2.x; }
    public static boolean GlyphLiftHighButton(LinearOpMode op){ return  op.gamepad2.y; }
    public static boolean GlyphLiftMaxButton(LinearOpMode op){ return   op.gamepad2.b; }
    public static boolean GlyphGrabLockButton(LinearOpMode op){ return  op.gamepad2.right_bumper; }

    //Relic Capture controls
    public static double  RelicGrabTrigger(LinearOpMode op){ return     op.gamepad2.left_trigger; }
    public static boolean RelicLiftButton(LinearOpMode op){ return      op.gamepad2.dpad_up; }
    public static boolean RelicLowerButton(LinearOpMode op){ return     op.gamepad2.dpad_down; }
    public static boolean RelicGrabLockButton(LinearOpMode op){
        return op.gamepad2.left_bumper;
    }
    public static double  RelicArmStick(LinearOpMode op){
        return       op.gamepad2.left_stick_y;
    }



}
