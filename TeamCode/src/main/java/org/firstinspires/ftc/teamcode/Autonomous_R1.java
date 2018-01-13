package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Position R1", group="Linear Opmode")
//@Disabled

public class Autonomous_R1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BotConfig indianaGary = new BotConfig();
    private double jeweloffest = 0;
    private double columnOffset = 0;
    private static double Drive_Power = 0.5;


    //public static final String TAG = "Vuforia VuMark Sample";

    //OpenGLMatrix lastLocation = null;
    //VuforiaLocalizer vuforia;

    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    //VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    //VuforiaTrackable relicTemplate = relicTrackables.get(0);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indianaGary.drive.init(hardwareMap);
        indianaGary.myJewelArm.init(hardwareMap);
        indianaGary.myGlyphLifter.init(hardwareMap);

        //vuparameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";

        //vuparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //this.vuforia = ClassFactory.createVuforiaLocalizer(vuparameters);
        //relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        String Team_Color = "red";

        waitForStart();

        //relicTrackables.activate();

        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        //Autonomous Commands
        indianaGary.myGlyphLifter.Capture();
        indianaGary.myJewelArm.LowerArm();
        jeweloffest = indianaGary.myJewelArm.JewelKnock("blue");
        if (jeweloffest < 0){
            indianaGary.drive.Backward(Drive_Power, -jeweloffest);
        }
        if (jeweloffest > 0){
            indianaGary.drive.Forward(Drive_Power, jeweloffest);
        }
        indianaGary.myJewelArm.RaiseArm();

        indianaGary.drive.Forward(Drive_Power, 36 + jeweloffest + columnOffset);

        indianaGary.drive.TurnRight(90);

        indianaGary.drive.Forward(Drive_Power, 12);

        indianaGary.myGlyphLifter.Release();
        indianaGary.drive.Backward(Drive_Power, 2);
    }

    //private double DecodeImage(){
        //Decode Image and offset final robot position to line up with correct column
        //Return offset distance in inches
        //double vuMarkColumnOffset = 0;
        //double columnRightOffset = -4; //Offset in inches from center column; negative is closer to bot's starting position
        //double columnLeftOffset = 4; //Offset in inches from center column; negative is closer to bot's starting position
        //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
        //    telemetry.addData("VuMark", "%s visible", vuMark);
        //    if (vuMark == RelicRecoveryVuMark.LEFT) {
        //        vuMarkColumnOffset = columnLeftOffset;
        //    }
        //    if (vuMark == RelicRecoveryVuMark.RIGHT) {
        //        vuMarkColumnOffset = columnRightOffset;
        //    }
        //}
        //else {
        //    telemetry.addData("VuMark", "not visible");
        //}

        //telemetry.update();
        //return vuMarkColumnOffset;
    //}

    //String format(OpenGLMatrix transformationMatrix) {
    //    return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    //}


}
