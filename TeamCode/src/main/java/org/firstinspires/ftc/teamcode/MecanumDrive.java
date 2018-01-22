package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public static double Turn_Power = 0.15;
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
        //motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public void StopWheels() {
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }

    public void Drive(LinearOpMode op, double power, double distance){
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double scaleFactor = 83.33;
        double drivePower = power;
        double heading = imu.getAngularOrientation().firstAngle;
        int startPosition = motorBL.getCurrentPosition();
        int endPosition = startPosition + (int)Math.round(distance * scaleFactor);
        motorBL.setTargetPosition(endPosition);
        motorBR.setTargetPosition(endPosition);
        motorBL.setPower(power);
        motorBR.setPower(power);
        motorFR.setPower(Math.signum(distance)*motorBL.getPower());
        motorFL.setPower(Math.signum(distance)*motorBL.getPower());
        while (motorBL.isBusy() && op.opModeIsActive()){
            if (Math.abs(motorBL.getCurrentPosition() - endPosition) < 500) {
                drivePower = 0.05 + power * Math.abs(motorBL.getCurrentPosition() - endPosition)/500 ;
            }
            motorBL.setPower(drivePower);
            motorBR.setPower(drivePower);
            motorFL.setPower(Math.signum(distance)*motorBL.getPower());
            motorFR.setPower(Math.signum(distance)*motorBR.getPower());
            op.telemetry.addData("Start", startPosition);
            op.telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
            op.telemetry.addData("Go to", endPosition);
            op.telemetry.addData("BR Position", motorBR.getCurrentPosition());
            op.telemetry.addData("BL Position", motorBL.getCurrentPosition());
            op.telemetry.update();
        }
        StopWheels();
        op.telemetry.addData("Start", startPosition);
        op.telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
        op.telemetry.addData("Go to", endPosition);
        op.telemetry.addData("BR Position", motorBR.getCurrentPosition());
        op.telemetry.addData("BL Position", motorBL.getCurrentPosition());
        op.telemetry.update();

    }

    public void Forward(LinearOpMode op, double power, double distance) {
        //Drive forward distance in inches. Use "scaleFactor" to convert inches to encoder values.

        double scaleFactor = 86.116;
        int startPosition = motorBL.getCurrentPosition();
        int endPosition = (int) (startPosition + (distance * scaleFactor));
        int threshold = 3;
        double drivePower = power;
         while (Math.signum(motorBL.getCurrentPosition() - endPosition) > threshold && op.opModeIsActive()) {
            motorFL.setPower(drivePower);
            motorFR.setPower(drivePower);
            motorBL.setPower(drivePower);
            motorBR.setPower(drivePower);
            if (Math.abs(motorBL.getCurrentPosition() - endPosition) < 500) {
                drivePower = power * Math.abs(motorBL.getCurrentPosition() - endPosition)/500 ;
            }
            op.telemetry.addData("Target Position", endPosition);
            op.telemetry.addData("Current Position", motorBL.getCurrentPosition());
            op.telemetry.addData("Remaining Distance", Math.signum(motorBL.getCurrentPosition() - endPosition));
            op.telemetry.addData("Drive Power", drivePower);
            op.telemetry.update();
        }
        StopWheels();

    }

    public void Backward(LinearOpMode op, double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 86.116;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        while (motorBL.getCurrentPosition() > endPosition && op.opModeIsActive()) {
            motorFL.setPower(-power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(-power);
        }
        StopWheels();
    }

    public void Right(LinearOpMode op, double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 116.94;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        while (motorBL.getCurrentPosition() > endPosition && op.opModeIsActive()) {
            motorFL.setPower(power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(power);
        }
        StopWheels();
    }
    public void Left(LinearOpMode op, double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 116.94;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));
        while (motorBL.getCurrentPosition() < endPosition && op.opModeIsActive()) {
            motorFL.setPower(-power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(-power);
        }
        StopWheels();
    }

    public void Turn(LinearOpMode op, double Angle){
        // + is left, - is right
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double targetAngle;
        double turnPower = 0.4;
        double threshold = .1;
        double difference;
        if (Math.abs(initialAngle + Angle) > 180) {
            targetAngle = initialAngle + Angle - Math.signum(initialAngle + Angle)*360;
        } else {
            targetAngle = initialAngle + Angle;
        }
        difference = targetAngle - imu.getAngularOrientation().firstAngle;
        while (Math.abs(difference) > threshold && op.opModeIsActive()) {
            turnPower = Math.signum(difference)*Math.sqrt(Math.abs(difference/targetAngle))/3;
            if (Math.abs(difference) <= 180) {
                motorFL.setPower(-turnPower);
                motorFR.setPower(turnPower);
                motorBL.setPower(-turnPower);
                motorBR.setPower(turnPower);
            } else {
                motorFL.setPower(turnPower);
                motorFR.setPower(-turnPower);
                motorBL.setPower(turnPower);
                motorBR.setPower(-turnPower);
            }
            difference = targetAngle - imu.getAngularOrientation().firstAngle;
            op.telemetry.addData("Target Angle", targetAngle);
            op.telemetry.addData("Current Angle", imu.getAngularOrientation().firstAngle);
            op.telemetry.addData("Difference", difference);
            op.telemetry.addData("Power", turnPower);
            op.telemetry.update();
        }
        StopWheels();
    }

    public void TurnLeft(LinearOpMode op, double Angle){

        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle < (initialAngle + Angle) && op.opModeIsActive()) {
            motorFL.setPower(-Turn_Power);
            motorFR.setPower(Turn_Power);
            motorBL.setPower(-Turn_Power);
            motorBR.setPower(Turn_Power);
            op.telemetry.addData("Target Angle", Angle);
            op.telemetry.addData("Current Angle", imu.getAngularOrientation().firstAngle);
            op.telemetry.update();

        }
        StopWheels();
    }

    public void TurnRight(LinearOpMode op, double Angle){
        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle > (initialAngle - Angle) && op.opModeIsActive()) {
            motorFL.setPower(Turn_Power);
            motorFR.setPower(-Turn_Power);
            motorBL.setPower(Turn_Power);
            motorBR.setPower(-Turn_Power);
        }
        StopWheels();
    }

}
