package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Position R1", group="Linear Opmode")
//@Disabled

public class Autonomous_R1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private static double Drive_Power = 0.25;

    private DcMotor motorLift = null;
    private Servo GrabberR = null;
    private Servo GrabberL = null;

    private ColorSensor colorSensorF;    // Hardware Device Object
    private ColorSensor colorSensorB;
    private Servo JewelArm = null;

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFL = hardwareMap.dcMotor.get("motor_fl");
        //motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR = hardwareMap.dcMotor.get("motor_fr");
        //motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL = hardwareMap.dcMotor.get("motor_bl");
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR = hardwareMap.dcMotor.get("motor_br");
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift  = hardwareMap.dcMotor.get("glyph_lifter");
        GrabberL = hardwareMap.servo.get("Glyph_Pad_Left");
        GrabberR = hardwareMap.servo.get("Glyph_Pad_Right");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorSensorF = hardwareMap.get(ColorSensor.class, "sensor_colorF");
        colorSensorB = hardwareMap.get(ColorSensor.class, "sensor_colorB");
        colorSensorB.setI2cAddress(I2cAddr.create8bit(0x3a));
        JewelArm = hardwareMap.servo.get("JewelArm");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        colorSensorB.enableLed(false);
        colorSensorF.enableLed(false);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        String Team_Color = "red";
        Double JewelOffset;
        Double ColumnOffset;
        JewelArm.setPosition(0.37);

        waitForStart();
        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        //Autonomous Commands
        ColumnOffset = DecodeImage();
        GlyphCapture();
        JewelOffset = JewelKnock(Team_Color);
        DriveBackward(Drive_Power, 36 + JewelOffset + ColumnOffset);
        TurnLeft(90);
        DriveForward(Drive_Power, 12);
        GlyphRelease();
        DriveBackward(Drive_Power, 2);


    }

    private double DecodeImage(){
        //Decode Image and offset final robot position to line up with correct column
        //Return offset distance in inches
        return 0;
    }

    private void StopWheels() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        sleep(500);
    }


    private void DriveForward(double power, double distance) {
        //Drive forward distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 89.17;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));
        telemetry.addData("Driving Forward: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();

        while (motorBL.getCurrentPosition() < endPosition) {
            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);
        }
        StopWheels();
    }

    private void DriveBackward(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 89.17;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        telemetry.addData("Driving Backward: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();
        while (motorBL.getCurrentPosition() > endPosition) {
            motorFL.setPower(-power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(-power);
        }
        StopWheels();
    }

    private void DriveRight(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 178.34;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        telemetry.addData("Driving Right: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();
        while (motorBL.getCurrentPosition() > endPosition) {
            motorFL.setPower(power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(power);
        }
        StopWheels();
    }
    private void DriveLeft(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 178.34;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));
        telemetry.addData("Driving Left: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();
        while (motorBL.getCurrentPosition() < endPosition) {
            motorFL.setPower(-power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(-power);
        }
        StopWheels();
    }
    public void TurnRight(double Angle){

        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle < (initialAngle + Angle)) {
            telemetry.addData("Turning Right: ", Angle);
            telemetry.addData("Start Angle: ", initialAngle);
            telemetry.addData("End Angle: ", initialAngle + Angle);
            telemetry.addData("Current Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            motorFL.setPower(-Drive_Power);
            motorFR.setPower(Drive_Power);
            motorBL.setPower(-Drive_Power);
            motorBR.setPower(Drive_Power);
        }
        StopWheels();
    }

    private void TurnLeft(double Angle){
        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle > (initialAngle - Angle)) {
            telemetry.addData("Turning Left: ", Angle);
            telemetry.addData("Start Angle: ", initialAngle);
            telemetry.addData("End Angle: ", initialAngle - Angle);
            telemetry.addData("Current Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            motorFL.setPower(Drive_Power);
            motorFR.setPower(-Drive_Power);
            motorBL.setPower(Drive_Power);
            motorBR.setPower(-Drive_Power);
        }
        StopWheels();
    }

    private void GlyphCapture() {
        GrabberL.setPosition(0.1);
        GrabberR.setPosition(0.9);
        motorLift.setPower(0.2);
        sleep(1000);
        motorLift.setPower(0.0);
    }

    private void GlyphRelease(){
        GrabberL.setPosition(0.5);
        GrabberR.setPosition(0.5);
    }

    private double JewelKnock(String myColor){
        //Knock off other team's jewel by driving foward or backwards based on color sensor
        //Return distance travelled in inches
        double Move_Distance = 0;
        //Lower jewel arm
        //JewelArm.setPosition(0.31);
        //Turn on LEDs
        colorSensorF.enableLed(true);
        colorSensorB.enableLed(true);
        //Give time for jewel arm to move and color sensor to read values
        sleep(1000);
        //Knock off the jewel
        if (ColorRedTestFront() == 0) {
            Move_Distance = 0;
        }
        if (ColorRedTestFront() == 1) {
            Move_Distance = -2;
            DriveBackward(Drive_Power, -Move_Distance);
        }
        if (ColorRedTestFront() == 2) {
            Move_Distance = 2;
            DriveForward(Drive_Power, Move_Distance);
        }


        //Raise jewel arm
        //JewelArm.setPosition(0.37);
        //Turn off LEDs
        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);

        return Move_Distance;
    }

    private int ColorRedTestFront() {
        int result = 0;
        telemetry.addData("Clear(F/B)", Integer.toString(colorSensorF.alpha()) + "/" + Integer.toString(colorSensorB.alpha()));
        telemetry.addData("Red(F/B)  ", Integer.toString(colorSensorF.red()) + "/" + Integer.toString(colorSensorB.red()));
        telemetry.addData("Green(F/B)", Integer.toString(colorSensorF.green()) + "/" + Integer.toString(colorSensorB.green()));
        telemetry.addData("Blue(F/B) ", Integer.toString(colorSensorF.blue()) + "/" + Integer.toString(colorSensorB.blue()));
        if (colorSensorF.red() > 4) {
            telemetry.addData("Move Back", "");
            result = 1;
        } else {
            if (colorSensorB.red() > 4) {
                telemetry.addData("Move Forward", "");
                result = 2;
            } else {
                if (colorSensorF.blue() > 4) {
                    telemetry.addData("Move Forward", "");
                    result = 2;
                } else {
                    if (colorSensorB.blue() > 4) {
                        telemetry.addData("Move Backward", "");
                        result = 1;
                    } else {
                        if (colorSensorF.red() > 0) {
                            telemetry.addData("Move Back", "");
                            result = 1;
                        } else {
                            if (colorSensorB.red() > 0) {
                                telemetry.addData("Move Forward", "");
                                result = 2;
                            } else {
                                if (colorSensorF.blue() > 0) {
                                    telemetry.addData("Move Forward", "");
                                    result = 2;
                                } else {
                                    if (colorSensorB.blue() > 0) {
                                        telemetry.addData("Move Backward", "");
                                        result = 1;
                                    } else {
                                        telemetry.addData("Don't Move", "");
                                        result = 0;
                                    }
                                }
                            }
                        }

                    }
                }

            }

        }
        telemetry.update();
        return result;

    }
}
