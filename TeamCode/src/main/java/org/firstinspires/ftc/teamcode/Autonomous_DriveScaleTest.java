package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Drive Scale Test", group="Tests")
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
        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();

        indianaGary.drive.initAuto(hardwareMap);
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

            //jewelOffset = indianaGary.myJewelArm.JewelKnock("red");
            jewelOffset = -2;
            if (jewelOffset < 0){
                indianaGary.drive.Drive(this, Drive_Power, jewelOffset, 10);
            }
            if (jewelOffset > 0){
                indianaGary.drive.Drive(this, Drive_Power, jewelOffset, 10);
            }
            //indianaGary.myJewelArm.RaiseArm();
             indianaGary.drive.Drive(this, 0.3, 36 - jewelOffset + columnOffset, 15);
            sleep(5000);
        //indianaGary.drive.motorBL.setTargetPosition(500);
        //indianaGary.drive.motorBL.setPower(0.2);


        //indianaGary.myGlyphLifter.Release();

    }

}
