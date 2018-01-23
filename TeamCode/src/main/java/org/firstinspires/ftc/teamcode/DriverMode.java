// tested 2017-11-05 AVP
/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive TeleOp for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Mode", group="Linear OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private BotConfig indianaGary = new BotConfig();



    @Override
    public void runOpMode() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();
        indianaGary.InitAll(hardwareMap);
        indianaGary.drive.motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indianaGary.drive.motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set toggle initial states
        boolean rtTogglePressed = false;
        boolean rtToggleReleased = true;
        boolean ltTogglePressed = false;
        boolean ltToggleReleased = true;
        boolean autoLift = false; //Used to enable auto motion of Glyph Lifter to preset Positions
        double DRIVE_POWER_MAX_LOW = 0.3; //Maximum drive power with not throttle

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();

            indianaGary.drive.DriveControl(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

            //Glyph Grabber Control
            if (!indianaGary.myGlyphLifter.GRAB_LOCKED) {
                indianaGary.myGlyphLifter.GrabberControl(gamepad2.right_trigger);
            }
            if (rtTogglePressed) {
                rtToggleReleased = false;
            } else {
                rtToggleReleased = true;
            }
            rtTogglePressed = gamepad2.right_bumper;
            if (rtToggleReleased){
                if (gamepad2.right_bumper && !indianaGary.myGlyphLifter.GRAB_LOCKED){
                    indianaGary.myGlyphLifter.Grab();
                } else {
                    if (gamepad2.right_bumper && indianaGary.myGlyphLifter.GRAB_LOCKED) {
                        indianaGary.myGlyphLifter.Release();
                    }
                }
            }

            //Glyph Lifter Control
            if (gamepad2.right_stick_y != 0) {
                autoLift = false;
                indianaGary.myGlyphLifter.LifterControl(-gamepad2.right_stick_y);
                //Turn off auto motion as soon as left stick is moved
                telemetry.addData("Lifter at Position: ", indianaGary.myGlyphLifter.motorLift.getCurrentPosition());
                telemetry.update();
            }else {
                //Go to preset positions when corresponding button is pressed
                if (gamepad2.a) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_1);
                }
                if (gamepad2.x) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_2);
                }
                if (gamepad2.y) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_3);
                }
                if (gamepad2.b) {
                    autoLift = true;
                    indianaGary.myGlyphLifter.GotoPresetPosition(indianaGary.myGlyphLifter.POS_MAX);
                }
                if (!autoLift) {
                    indianaGary.myGlyphLifter.motorLift.setPower(0);
                }
            }

            //Relic Capture Control
            indianaGary.myRelicArm.ArmExtension(-gamepad2.left_stick_y);
            telemetry.addData("Arm Position", indianaGary.myRelicArm.motorRelicArm.getCurrentPosition());
            telemetry.update();

            //The check for Relic Grabber's position is to prevent the Lifter from accidentally
            //lifting when the relic isn't being grabbed
            if (gamepad2.dpad_up && indianaGary.myRelicArm.relicGrab.getPosition() > 0.25) {
                indianaGary.myRelicArm.Lift();
            }
            if (gamepad2.dpad_down) {
                indianaGary.myRelicArm.Lower();
            }

            if (!indianaGary.myRelicArm.LOCKED) {
                indianaGary.myRelicArm.GrabberControl(gamepad2.left_trigger);
            }

            if (ltTogglePressed) {
                ltToggleReleased = false;
            } else {
                ltToggleReleased = true;
            }
            ltTogglePressed = gamepad2.left_bumper;
            if (ltToggleReleased){
                if (gamepad2.left_bumper && !indianaGary.myRelicArm.LOCKED) {
                    indianaGary.myRelicArm.Grab();
                } else {
                    if (gamepad2.left_bumper && indianaGary.myRelicArm.LOCKED){
                        indianaGary.myRelicArm.Release();
                    }

                }
            }
        }
    }
}