package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Blue Left", group="Linear Opmode - After Auto")
//@Disabled

public class Autonomous_B2 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BotConfig indianaGary = new BotConfig();
    private double jewelOffset = 0;
    private double columnOffset = 0;
    private static double Drive_Power = 0.3;
    private double otf_correction = 0;
    private boolean buttonPressed;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        telemetry.addLine("Status: Initializing. Please Wait...");
        telemetry.update();
        indianaGary.InitAuto(hardwareMap);
        //indianaGary.InitServos(hardwareMap);
        //telemetry.addLine("Servos Initialized");
        //indianaGary.InitMotors(hardwareMap);
        //telemetry.addLine("Motors Initialized");
        //indianaGary.InitSensors(hardwareMap);
        //telemetry.addLine("Sensors Initialized");
        AutoTransitioner.transitionOnStop(this, "Driver Mode");

        String Team_Color = "blue";

        telemetry.addLine("Status Initialized");
        Telemetry.Item otf = telemetry.addData("OTF Correction (use D-pad to change)", 0);
        while (!opModeIsActive()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("OTF Correction (use D-pad to change)", otf_correction);
            telemetry.update();
            //allow an "on-the-fly" correction to be applied to the drive distance
            //use D-pad up and down to change
            if (!buttonPressed){
                if (gamepad1.dpad_up) {
                    buttonPressed = true;
                    otf_correction = otf_correction + 0.1;
                }
                if (gamepad1.dpad_down) {
                    buttonPressed = true;
                    otf_correction = otf_correction - 0.1;
                }
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                buttonPressed = false;
            }
        }

        waitForStart();

        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        //Autonomous Commands
        indianaGary.myGlyphLifter.Capture(this);
        columnOffset = indianaGary.myVuMark.DecodeImage(this);
        indianaGary.myJewelArm.LowerArm();
        jewelOffset = indianaGary.myJewelArm.JewelKnock("red");
        if (jewelOffset < 0) {
            telemetry.addLine("Jewel on FRONT");
        } else {
            if (jewelOffset > 0) {
                telemetry.addLine("Jewel on BACK");
            } else {
                telemetry.addLine("Jewel UNKNOWN");
            }
        }
        indianaGary.drive.Drive(this, Drive_Power, jewelOffset, 5);
        indianaGary.myJewelArm.RaiseArm();
        indianaGary.drive.Drive(this, Drive_Power, -(15 - jewelOffset), 15);
        indianaGary.drive.Turn(this,90, 10);
        indianaGary.drive.Drive(this, Drive_Power, 8 + columnOffset  + otf_correction, 15);
        indianaGary.drive.Turn(this,90, 10);
        indianaGary.drive.Drive(this, Drive_Power, 5, 4);
        indianaGary.myGlyphLifter.Release();
        indianaGary.myGlyphLifter.GotoPresetPosition(0);
        indianaGary.myGlyphLifter.grabberL.setPosition(indianaGary.myGlyphLifter.GRABBER_OPEN);
        indianaGary.myGlyphLifter.grabberR.setPosition(indianaGary.myGlyphLifter.GRABBER_OPEN);
    }
}
