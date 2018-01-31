package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Lyesome on 2018-01-13.
 * This class contains all the methods for controlling the Jewel Arm
 */

public class JewelArm {
    HardwareMap myHWMap;
    private ColorSensor colorSensorF;    // Hardware Device Object
    private ColorSensor colorSensorB;
    private Servo jewelArm = null;
    final double KNOCK_DISTANCE = 2;
    final double UP_POSITION = 0.0;
    final double DOWN_POSITION = 0.60;

    public void JewelArm(){ //constructor
    }

    //Method to initialize all the Jewel Arm hardware
    public void init(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        //Initialize the Jewel Arm
        jewelArm = myHWMap.servo.get("servo_jewel_arm");
        jewelArm.setDirection(Servo.Direction.REVERSE);
        jewelArm.setPosition(UP_POSITION);

        //Initialize both color sensors (F = Front, B = Back)
        colorSensorF = myHWMap.get(ColorSensor.class, "sensor_color_f");
        colorSensorB = myHWMap.get(ColorSensor.class, "sensor_color_b");
        colorSensorB.setI2cAddress(I2cAddr.create8bit(0x3a)); //assign back color sensor to altered address
        //Turn off LEDS to save power
        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);
    }

    public void initSensors(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;
        //Initialize both color sensors (F = Front, B = Back)
        colorSensorF = myHWMap.get(ColorSensor.class, "sensor_color_f");
        colorSensorB = myHWMap.get(ColorSensor.class, "sensor_color_b");
        colorSensorB.setI2cAddress(I2cAddr.create8bit(0x3a)); //assign back color sensor to altered address
        //Turn off LEDS to save power
        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);
    }

    public void initServos(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;
        jewelArm = myHWMap.servo.get("servo_jewel_arm");
        jewelArm.setDirection(Servo.Direction.REVERSE);
        jewelArm.setPosition(UP_POSITION);
    }

    //Method to automatically raise the Jewel Arm
    public void RaiseArm() {
        //Turn off LEDS to save power
        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);

        jewelArm.setPosition(UP_POSITION);
    }

    //Method to automatically lower the Jewel Arm
    public  void LowerArm(){
        jewelArm.setPosition(DOWN_POSITION);

        //Turn on LEDs to improve reading color values
        colorSensorF.enableLed(true);
        colorSensorB.enableLed(true);
    }

    //Method to indicate which direction to move to knock off other team's jewel
    public double JewelKnock(String color){ //color of jewel to knock off
        double Move_Distance = 0;

        try { //pause for a moment to give color sensors time to detect color
            Thread.sleep(500);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

        if(color == "blue") { //If on red team, knock off blue jewel
            if (ColorRedTestFront() == 0) {
                //Do not move in either direction if results of test are inconclusive
                Move_Distance = 0;
            }
            if (ColorRedTestFront() == 1) {
                //Indicate to move backwards if test indicates red jewel is in front position
                Move_Distance = -KNOCK_DISTANCE;
            }
            if (ColorRedTestFront() == 2) {
                //Indicate to move forward if test indicates red jewel is in back position
                Move_Distance = KNOCK_DISTANCE;
            }
        }

        if(color == "red") { //If on blue team, knock off red jewel
            if (ColorRedTestFront() == 0) {
                //Do not move in either direction if results of test are inconclusive
                Move_Distance = 0;
            }
            if (ColorRedTestFront() == 1) {
                //Indicate to move forward if test indicates red jewel is in front position
                Move_Distance = KNOCK_DISTANCE;
            }
            if (ColorRedTestFront() == 2) {
                //Indicate to move backwards if test indicates red jewel is in back position
                Move_Distance = -KNOCK_DISTANCE;
            }
        }
        return Move_Distance;
    }

    /*Method test if red jewel is in the front position
    This method return three possible results:
        0 = Inconclusive
        1 = Red in Front position
        2 = Red in Back position
    */
    private int ColorRedTestFront() {
        int result;
        //Jewels can color readings up to 4 of the opposite color
        //To start, only compare front and back color sensor readings above 4
        //to catch when only one jewel is detected.
        //A reading above 4 positively identifies the jewel's color
        if (colorSensorF.red() > 4) {
            result = 1;
        } else {
            if (colorSensorB.red() > 4) {
                result = 2;
            } else {
                if (colorSensorF.blue() > 4) {
                    result = 2;
                } else {
                    if (colorSensorB.blue() > 4) {
                        result = 1;
                    } else {
                        //If no readings are above 4, increase sensitivity to 1
                        if (colorSensorF.red() > 0) {
                            result = 1;
                        } else {
                            if (colorSensorB.red() > 0) {
                                result = 2;
                            } else {
                                if (colorSensorF.blue() > 0) {
                                    result = 2;
                                } else {
                                    if (colorSensorB.blue() > 0) {
                                        result = 1;
                                    } else {
                                        //If no readings above 0, give up
                                        result = 0;
                                    }
                                }
                            }
                        }

                    }
                }

            }

        }
        return result;

    }



}
