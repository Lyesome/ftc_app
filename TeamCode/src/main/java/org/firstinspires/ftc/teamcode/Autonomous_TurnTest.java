package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

/**
 * Created by maris on 2018-01-03.
 */
@Autonomous(name="Indiana Gary - Turn Test", group="Linear Opmode")
//@Disabled

public class Autonomous_TurnTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private static double Drive_Power = 0.5;

    // The IMU sensor object
    BNO055IMU imu;

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

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

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

        waitForStart();
        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        //Autonomous Commands
        TurnLeft(90);
        TurnRight(90);

    }


    private void StopWheels() {
        //telemetry.addData("Wheels: ", "Stopped");
        //telemetry.update();
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        sleep(1000);
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
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (endPosition - motorBL.getCurrentPosition())) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
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
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (motorBL.getCurrentPosition() - endPosition)) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
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
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (motorBL.getCurrentPosition() - endPosition)) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
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
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (endPosition - motorBL.getCurrentPosition())) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
    }

    public void TurnRight(double Angle){

        double initialAngle = imu.getAngularOrientation().firstAngle;
        while (imu.getAngularOrientation().firstAngle < initialAngle + Angle) {
            telemetry.addData("Turning Right: ", Angle);
            telemetry.addData("Start Angle: ", initialAngle);
            telemetry.addData("End Angle: ", initialAngle + Angle);
            telemetry.update();
            motorFL.setPower(1.0);
            motorFR.setPower(-1.0);
            motorBL.setPower(1.0);
            motorBR.setPower(-1.0);
        }
        StopWheels();
    }
    private void TurnLeft(double Angle){
        double initialAngle = imu.getAngularOrientation().firstAngle;
        telemetry.addData("Turning Left: ", Angle);
        telemetry.addData("Start Angle: ", initialAngle);
        telemetry.addData("End Angle: ", initialAngle + Angle);
        telemetry.update();
        while (imu.getAngularOrientation().firstAngle >= initialAngle - Angle) {
            motorFL.setPower(-1.0);
            motorFR.setPower(1.0);
            motorBL.setPower(-1.0);
            motorBR.setPower(1.0);
        }
        StopWheels();
    }

}
