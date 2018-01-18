package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by maris on 2018-01-13.
 */

public class BotConfig {
    //Add components to robot build
    MecanumDrive drive = new MecanumDrive();

    JewelArm myJewelArm = new JewelArm();

    RelicArm myRelicArm = new RelicArm();

    //VuMarkDecoder myVuMark = new VuMarkDecoder();

    GlyphLifter myGlyphLifter = new GlyphLifter();

    public BotConfig() { // constructor

    }

    public void InitAll(HardwareMap myNewHWMap){
        myJewelArm.init(myNewHWMap); //need to initialize to prevent arm from dropping
        myRelicArm.init(myNewHWMap);
        myGlyphLifter.init(myNewHWMap);
        drive.init(myNewHWMap);

    }
}
