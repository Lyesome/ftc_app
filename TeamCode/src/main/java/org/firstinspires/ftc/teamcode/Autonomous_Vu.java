package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - VuTest", group="Tests")
//@Disabled

public class Autonomous_Vu extends LinearOpMode {
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
        indianaGary.InitAuto(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        String Team_Color = "red";

        waitForStart();
        while (opModeIsActive()) {
            runtime.reset();

            // run until the end of the match

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();


            //Autonomous Commands


            columnOffset = indianaGary.myVuMark.DecodeImage(this);
            telemetry.addData("Column Offset", columnOffset);
            telemetry.update();
        }
    }


}
