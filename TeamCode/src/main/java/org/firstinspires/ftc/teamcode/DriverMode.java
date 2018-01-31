

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lyesome on 2018-01-13.
 * This file contains all the instructions for controlling the robot in Teleop mode.
 */

@TeleOp(name="Driver Mode", group="Linear OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //creates a new robot named indianaGary
    private BotConfig indianaGary = new BotConfig();



    @Override
    public void runOpMode() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        //Provide warning for drivers not to hit play until initializing is complete.
        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();
        //indianaGary.InitTele(hardwareMap);
        indianaGary.InitServos(hardwareMap);
        indianaGary.InitMotors(hardwareMap);
        //for manual driving encoder is not needed in the drive motors.

        //Set toggle initial states
        boolean rtTogglePressed = false;
        boolean rtToggleReleased = true;
        boolean ltTogglePressed = false;
        boolean ltToggleReleased = true;
        boolean autoLift = false; //Used to enable auto motion of Glyph Lifter to preset Positions
        //tell drivers that initializing is now complete
        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        indianaGary.myRelicArm.relicGrab.setPosition(indianaGary.myRelicArm.RELIC_GRAB_OPEN);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lifter at Position", indianaGary.myGlyphLifter.motorLift.getCurrentPosition());
            telemetry.addData("Arm at Position", indianaGary.myRelicArm.motorRelicArm.getCurrentPosition());
            telemetry.update();

            //pass controls to the drive control method.
            indianaGary.drive.DriveControl(BotControls.DriveYStick(this),
                                           BotControls.DriveXStick(this),
                                           BotControls.TurnStick(this),
                                           BotControls.DriveThrottle(this));

            //Glyph Grabber Control
            if (!indianaGary.myGlyphLifter.GRAB_LOCKED) { //only allow control of grabber if not locked.
                indianaGary.myGlyphLifter.GrabberControl(BotControls.GlyphGrabTrigger(this));
            }

            if (rtTogglePressed) {
                rtToggleReleased = false;
            } else {
                //Track when toggle button is released
                rtToggleReleased = true;
            }
            rtTogglePressed = BotControls.GlyphGrabLockButton(this);
            if (rtToggleReleased){ //only do something if toggle is pressed after being released
                if (BotControls.GlyphGrabLockButton(this) && !indianaGary.myGlyphLifter.GRAB_LOCKED){
                    indianaGary.myGlyphLifter.Grab();
                } else {
                    if (BotControls.GlyphGrabLockButton(this) && indianaGary.myGlyphLifter.GRAB_LOCKED) {
                        indianaGary.myGlyphLifter.Release();
                    }
                }
            }

            //Glyph Lifter Control
            if (BotControls.GlyphLiftStick(this) != 0) {
                autoLift = false; //turn off auto motion flag if manual control starts
                indianaGary.myGlyphLifter.LifterControl(-BotControls.GlyphLiftStick(this));
            }else {
                //Go to preset positions when corresponding button is pressed
                if (BotControls.GlyphLiftMinButton(this)) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_1);
                }
                if (BotControls.GlyphLiftLowButton(this)) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_2);
                }
                if (BotControls.GlyphLiftHighButton(this)) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_3);
                }
                if (BotControls.GlyphLiftMaxButton(this)) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_MAX);
                }
                if (!autoLift) {
                    //turn off power to lifter if auto motion is off and no manual control
                    indianaGary.myGlyphLifter.motorLift.setPower(0);
                }
            }

            //Relic Capture Control
            indianaGary.myRelicArm.ArmExtension(-BotControls.RelicArmStick(this));

            //The check for Relic Grabber's position is to prevent the Lifter from accidentally
            //lifting when the relic isn't being grabbed
            if (BotControls.RelicLiftButton(this) && indianaGary.myRelicArm.relicGrab.getPosition() > 0.25) {
                indianaGary.myRelicArm.Lift();
            }
            if (BotControls.RelicLowerButton(this)) {
                indianaGary.myRelicArm.Lower();
            }

            if (!indianaGary.myRelicArm.LOCKED) {
                indianaGary.myRelicArm.GrabberControl(BotControls.RelicGrabTrigger(this));
            }

            if (ltTogglePressed) {
                ltToggleReleased = false;
            } else {
                //Track when toggle button is released
                ltToggleReleased = true;
            }
            ltTogglePressed = BotControls.RelicGrabLockButton(this);
            if (ltToggleReleased){ //only do something if toggle is pressed after being released
                if (BotControls.RelicGrabLockButton(this) && !indianaGary.myRelicArm.LOCKED) {
                    indianaGary.myRelicArm.Grab();
                } else {
                    if (BotControls.RelicGrabLockButton(this) && indianaGary.myRelicArm.LOCKED){
                        indianaGary.myRelicArm.Release();
                    }

                }
            }
        }
    }
}