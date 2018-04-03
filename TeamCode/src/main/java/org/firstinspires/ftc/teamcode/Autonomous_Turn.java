package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Turn Test", group="Tests")
//@Disabled

public class Autonomous_Turn extends LinearOpMode {
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
        telemetry.addData("Heading", indianaGary.drive.imu.getAngularOrientation().firstAngle);
        telemetry.update();


        String Team_Color = "blue";

        waitForStart();

        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("BL wheel starting at", indianaGary.drive.motorBL.getCurrentPosition());
        telemetry.update();


        //Autonomous Commands
        indianaGary.drive.Turn(this, 90, 10);
        sleep(1000);
        indianaGary.drive.Turn(this, -90, 10);
        sleep(1000);
    }

}
