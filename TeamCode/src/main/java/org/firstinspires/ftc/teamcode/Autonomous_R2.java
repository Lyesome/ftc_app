package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
    private double jeweloffest = 0;
    private double columnOffset = 0;
    private static double Drive_Power = 0.2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indianaGary.drive.init(hardwareMap);
        indianaGary.myJewelArm.init(hardwareMap);
        indianaGary.myGlyphLifter.init(hardwareMap);
        indianaGary.myVuMark.init(hardwareMap);



        String Team_Color = "red";

        waitForStart();

        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        //Autonomous Commands
        indianaGary.myGlyphLifter.Capture();
        //columnOffset = indianaGary.myVuMark.DecodeImage();
        indianaGary.myJewelArm.LowerArm();
        jeweloffest = indianaGary.myJewelArm.JewelKnock("blue");
        if (jeweloffest < 0){
            indianaGary.drive.Backward(Drive_Power, -jeweloffest);
        }
        if (jeweloffest > 0){
            indianaGary.drive.Forward(Drive_Power, jeweloffest);
        }
        indianaGary.myJewelArm.RaiseArm();

        indianaGary.drive.Forward(Drive_Power, 24 - jeweloffest);

        indianaGary.drive.TurnLeft(90);

        indianaGary.drive.Forward(Drive_Power, 12 + columnOffset);

        indianaGary.drive.TurnRight(90);

        indianaGary.drive.Forward(Drive_Power, 3);

        indianaGary.myGlyphLifter.Release();
        indianaGary.drive.Backward(Drive_Power, 2);
    }




}
