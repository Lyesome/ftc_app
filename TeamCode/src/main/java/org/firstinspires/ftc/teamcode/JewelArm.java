package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by maris on 2018-01-13.
 */

public class JewelArm {
    private ColorSensor colorSensorF;    // Hardware Device Object
    private ColorSensor colorSensorB;
    private Servo jewelArm = null;
    HardwareMap myHWMap;
    final double KNOCK_DISTANCE = 2;
    final double UP_POSITION = 0.25;
    final double DOWN_POSITION = 0.88;

    public void JewelArm(){

    }
    public void init(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        colorSensorF = myHWMap.get(ColorSensor.class, "sensor_color_f");
        colorSensorB = myHWMap.get(ColorSensor.class, "sensor_color_b");
        colorSensorB.setI2cAddress(I2cAddr.create8bit(0x3a));
        jewelArm = myHWMap.servo.get("servo_jewel_arm");
        jewelArm.setDirection(Servo.Direction.REVERSE);

        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);

        jewelArm.setPosition(UP_POSITION);
    }

    public void RaiseArm() {
        //turn off LEDS
        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);

        // Raise Jewel Arm
        jewelArm.setPosition(UP_POSITION);
    }

    public  void LowerArm(){
        jewelArm.setPosition(DOWN_POSITION);
        colorSensorF.enableLed(true);
        colorSensorB.enableLed(true);


    }

    public double JewelKnock(String color){
        double Move_Distance = 0;

        try {
            Thread.sleep(500);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

        if(color == "blue") {
            if (ColorRedTestFront() == 0) {
                Move_Distance = 0;
            }
            if (ColorRedTestFront() == 1) {
                Move_Distance = -KNOCK_DISTANCE;
            }
            if (ColorRedTestFront() == 2) {
                Move_Distance = KNOCK_DISTANCE;
            }
        }

        if(color == "red") {
            if (ColorRedTestFront() == 0) {
                Move_Distance = 0;
            }
            if (ColorRedTestFront() == 1) {
                Move_Distance = KNOCK_DISTANCE;
            }
            if (ColorRedTestFront() == 2) {
                Move_Distance = -KNOCK_DISTANCE;
            }

        }
        return Move_Distance;
    }

    private int ColorRedTestFront() {
        int result = 0;
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
