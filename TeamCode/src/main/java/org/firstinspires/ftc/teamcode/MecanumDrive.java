package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by maris on 2018-01-13.
 */

public class MecanumDrive {
    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;
    public static double Drive_Power = 0.5;
    public static double Turn_Power = 0.2;
    // IMU sensor object
    BNO055IMU imu;

    HardwareMap myHWMap;

    public MecanumDrive(){

    }

    public void init(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        motorFL  = myHWMap.dcMotor.get("motor_fl");
        motorFR  = myHWMap.dcMotor.get("motor_fr");
        motorBL  = myHWMap.dcMotor.get("motor_bl");
        motorBR  = myHWMap.dcMotor.get("motor_br");



        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        imu = myHWMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    private void StopWheels() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }


    public void Forward(double power, double distance) {
        //Drive forward distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 89.17;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));

        while (motorBL.getCurrentPosition() < endPosition) {
            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);
        }
        StopWheels();
    }

    public void Backward(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 89.17;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        while (motorBL.getCurrentPosition() > endPosition) {
            motorFL.setPower(-power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(-power);
        }
        StopWheels();
    }

    public void Right(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 178.34;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        while (motorBL.getCurrentPosition() > endPosition) {
            motorFL.setPower(power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(power);
        }
        StopWheels();
    }
    public void Left(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 178.34;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));
        while (motorBL.getCurrentPosition() < endPosition) {
            motorFL.setPower(-power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(-power);
        }
        StopWheels();
    }
    public void TurnLeft(double Angle){

        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle < (initialAngle + Angle)) {
            motorFL.setPower(-Turn_Power);
            motorFR.setPower(Turn_Power);
            motorBL.setPower(-Turn_Power);
            motorBR.setPower(Turn_Power);
        }
        StopWheels();
    }

    public void TurnRight(double Angle){
        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle > (initialAngle - Angle)) {
            motorFL.setPower(Turn_Power);
            motorFR.setPower(-Turn_Power);
            motorBL.setPower(Turn_Power);
            motorBR.setPower(-Turn_Power);
        }
        StopWheels();
    }

}
