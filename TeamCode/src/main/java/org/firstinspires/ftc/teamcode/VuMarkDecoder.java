package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by maris on 2018-01-13.
 */

public class VuMarkDecoder {

    HardwareMap myHWMap;

    public static final String TAG = "Vuforia VuMark Sample";

    VuforiaLocalizer vuforia;

    OpenGLMatrix lastLocation = null;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;



    public void VuMarkDecoder() {

    }

    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;
        int cameraMonitorViewId = myHWMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHWMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuparameters.vuforiaLicenseKey = "AfXDkbT/////AAAAGUWkW5XORUDZk0pzMnL5JlVLvMH8yBho/fstQbUOWSs+KpTGzK7G45wHLlm81SXcl71Youk9yLvlN8hblV/+U0s5aamvYKWA71dh8aiXVKYqoDyF5V70BbEXcfUXOcRphDBLUpnCLgVYPxr837L4Yc8RHPVlEYXAtbYKJAvjnMZurqHTSvQG4G/XV5QcFJaJPFyP9zC/sPlkGgdg/xDxYzkABnxDJFTlIKePvpgxCcednmCT6bG/hE5ZeuBxNtC7kWI0xqrG5L90Pq0UZ64Y87esm7DujazZ9YrRVkpNRXcM80kSm+27BrpPvubNeT1lxpRVAzsxZX5AXPAnrHUO3dMMx66HqXzp6X82OgLcHEL1";
        vuparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(vuparameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

    }

    public double DecodeImage(LinearOpMode op){
        //Decode Image and offset final robot position to line up with correct column
        //Return offset distance in inches
        double vuMarkColumnOffset = 0;
        double columnRightOffset = -7.5; //Offset in inches from center column; negative is closer to bot's starting position
        double columnLeftOffset = 7.5; //Offset in inches from center column; negative is closer to bot's starting position
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    /* Found an instance of the template. In the actual game, you will probably
                     * loop until this condition occurs, then move on to act accordingly depending
                     * on which VuMark was visible. */
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                vuMarkColumnOffset = columnLeftOffset;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                vuMarkColumnOffset = columnRightOffset;
           }
        }
        else {

        }
        op.telemetry.addData("Column Offset", vuMarkColumnOffset);
        op.telemetry.update();
        return vuMarkColumnOffset;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}