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
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test_For_Drivers", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class Test_For_Drivers extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;

    public DcMotor motorLift = null;
    long StartPosition;
    long MaxPosition;



    private Servo GlyphPadLeft = null; //Robot Left
    private Servo GlyphPadRight = null; //Robot Right
    final double Glyph_Left_MIN  = 0.30;
    final double Glyph_Left_MAX  = 1;
    final double Glyph_Left_Release  = 0.45;

    final double Glyph_Right_MIN  = 0;
    final double Glyph_Right_MAX  = 0.7;
    final double Glyph_Right_Release  = 0.55;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorFL  = hardwareMap.dcMotor.get("motor_fl");
        motorFR  = hardwareMap.dcMotor.get("motor_fr");
        motorBL  = hardwareMap.dcMotor.get("motor_bl");
        motorBR  = hardwareMap.dcMotor.get("motor_br");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorFR.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorBR.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors

        motorLift  = hardwareMap.dcMotor.get("glyph_lifter");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StartPosition = motorLift.getCurrentPosition();
        MaxPosition = StartPosition + 1680; //1680
        motorLift.setDirection(DcMotor.Direction.REVERSE);

        GlyphPadLeft = hardwareMap.servo.get("Glyph_Pad_Left");
        GlyphPadRight = hardwareMap.servo.get("Glyph_Pad_Right");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // Mecanum Drive code
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            motorFL.setPower(v1);
            motorFR.setPower(v2);
            motorBL.setPower(v3);
            motorBR.setPower(v4);


            //Glyph Lifter
            if (gamepad2.left_stick_y > 0 && motorLift.getCurrentPosition() <= MaxPosition) {
                motorLift.setPower(gamepad2.left_stick_y);
            }else{
                if (gamepad2.left_stick_y < 0 && motorLift.getCurrentPosition() >= StartPosition) {
                    motorLift.setPower(gamepad2.left_stick_y * 0.7);
                }else{
                    motorLift.setPower(0);
                }
            }

            //Glyph pads
            GlyphPadLeft.setPosition(Range.clip(1-gamepad2.right_stick_y * 0.7 ,Glyph_Left_MIN,Glyph_Left_MAX));
            GlyphPadRight.setPosition(Range.clip(gamepad2.right_stick_y * 0.7,Glyph_Right_MIN,Glyph_Right_MAX));


        }
    }
}
