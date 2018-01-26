package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Red Right", group="Linear Opmode")
//@Disabled

public class Autonomous_R2 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BotConfig indianaGary = new BotConfig();
    private double jewelOffset = 0;
    private double columnOffset = 0;
    private static double Drive_Power = 0.3;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();
        indianaGary.InitAll(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        String Team_Color = "red";

        waitForStart();

        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        //Autonomous Commands
        indianaGary.myGlyphLifter.Capture();
        columnOffset = indianaGary.myVuMark.DecodeImage(this);
        indianaGary.myJewelArm.LowerArm();
        jewelOffset = indianaGary.myJewelArm.JewelKnock("blue");
        indianaGary.drive.Drive(this, Drive_Power, jewelOffset, 5);
        indianaGary.myJewelArm.RaiseArm();
        indianaGary.drive.Drive(this, Drive_Power, 28 - jewelOffset, 15);
        indianaGary.drive.Turn(this, 90);
        indianaGary.drive.Drive(this, Drive_Power, 12 + columnOffset, 15);
        indianaGary.drive.Turn(this, -90);
        indianaGary.drive.Drive(this, Drive_Power, 2, 3);
        indianaGary.myGlyphLifter.Release();
        indianaGary.myGlyphLifter.GotoPresetPosition(10);
        sleep(1000);  //give time for glyph to settle
        indianaGary.drive.Turn(this,180);
        indianaGary.drive.Drive(this, Drive_Power, -1, 5);
    }
}
