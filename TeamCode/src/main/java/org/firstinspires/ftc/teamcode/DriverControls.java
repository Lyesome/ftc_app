package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by maris on 2018-01-13.
 */

public class DriverControls {

    public DriverControls()  { // constructor

    }

    public static double TurnStick(LinearOpMode op){
        return op.gamepad1.right_stick_x;
    }
    public static double DriveStick(LinearOpMode op){
        return op.gamepad1.right_stick_y;
    }
    public static double GlyphGrabButton(LinearOpMode op){
        return op.gamepad1.right_stick_y;
    }

}
