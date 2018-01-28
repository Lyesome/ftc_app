package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Lyesome on 2018-01-13.
 * This class is used to initialize all of the components that make up the robot.
 */

public class BotConfig {
    //Add components to robot build
    MecanumDrive drive = new MecanumDrive();

    JewelArm myJewelArm = new JewelArm();

    RelicArm myRelicArm = new RelicArm();

    VuMarkDecoder myVuMark = new VuMarkDecoder();

    GlyphLifter myGlyphLifter = new GlyphLifter();

    public BotConfig() { // constructor
    }

    //Method to initialize all the Hardware
    public void InitAuto(HardwareMap myNewHWMap){
        myJewelArm.init(myNewHWMap); //need to initialize to prevent arm from dropping
        myGlyphLifter.init(myNewHWMap);
        myRelicArm.init(myNewHWMap);
        myVuMark.init(myNewHWMap);
        drive.initAuto(myNewHWMap);
    }
    public void InitTele(HardwareMap myNewHWMap){
        myJewelArm.init(myNewHWMap); //need to initialize to prevent arm from dropping
        myRelicArm.init(myNewHWMap);
        myGlyphLifter.init(myNewHWMap);
        drive.initTele(myNewHWMap);
    }
}
