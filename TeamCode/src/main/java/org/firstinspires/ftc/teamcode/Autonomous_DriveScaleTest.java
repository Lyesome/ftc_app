package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Drive Distance Scale Test", group="Tests")
//@Disabled

public class Autonomous_DriveScaleTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BotConfig indianaGary = new BotConfig();
    private double jewelOffset = 0;
    private double columnOffset = 0;
    private static double Drive_Power = 0.2;

    @Override
    public void runOpMode() {

        indianaGary.drive.init(hardwareMap);
        indianaGary.myJewelArm.init(hardwareMap);
        indianaGary.myGlyphLifter.init(hardwareMap);
        //indianaGary.myVuMark.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        String Team_Color = "blue";

        waitForStart();

        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("BL wheel starting at", indianaGary.drive.motorBL.getCurrentPosition());
        telemetry.update();


        //Autonomous Commands
        //indianaGary.myGlyphLifter.Capture();
        //columnOffset = indianaGary.myVuMark.DecodeImage();
        //indianaGary.myJewelArm.LowerArm();

            jewelOffset = indianaGary.myJewelArm.JewelKnock("red");
            if (jewelOffset < 0){
                indianaGary.drive.Backward(this, Drive_Power, -jewelOffset);
            }
            if (jewelOffset > 0){
                indianaGary.drive.Forward(this, Drive_Power, jewelOffset);
            }
            //indianaGary.myJewelArm.RaiseArm();
            indianaGary.drive.Forward(this, Drive_Power, 36 - jewelOffset + columnOffset);


        //indianaGary.drive.motorBL.setTargetPosition(500);
        //indianaGary.drive.motorBL.setPower(0.2);


        telemetry.addData("Drive Counts", indianaGary.drive.motorBL.getCurrentPosition());
        telemetry.update();

        //indianaGary.myGlyphLifter.Release();

    }

}
