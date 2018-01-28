package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the drive system using mecanum wheels
 */

public class MecanumDrive {
    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;
    public static double Turn_Power = 0.15;
    double DRIVE_POWER_MAX_LOW = 0.3; //Maximum drive power with not throttle
    // IMU sensor object (gyro)
    BNO055IMU imu;

    HardwareMap myHWMap;

    public MecanumDrive(){ //constructor
    }

    public void initAuto(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        //Initialize wheel motors
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

    public void initTele(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;

        //Initialize wheel motors
        motorFL = myHWMap.dcMotor.get("motor_fl");
        motorFR = myHWMap.dcMotor.get("motor_fr");
        motorBL = myHWMap.dcMotor.get("motor_bl");
        motorBR = myHWMap.dcMotor.get("motor_br");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**Method for manual control of drive system
    yStick controls forward and backward motion
    xStick controls lateral motion (strafe)
    turnStick control rotation (turn)
    Trigger controls the throttle (speed) */
    public void DriveControl(double yStick, double xStick, double turnStick, double trigger){
        double r = Math.hypot(xStick, yStick);
        double robotAngle = Math.atan2(-yStick, xStick) - Math.PI / 4;
        //Set minimum throttle value so the trigger does not need to be pressed to drive
        double trottle = trigger * (1-DRIVE_POWER_MAX_LOW) + DRIVE_POWER_MAX_LOW;
        //Cube the value of turnstick so there's more control over low turn speeds
        double rightX = Math.pow(turnStick, 3);
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        motorFL.setPower(v1*trottle);
        motorFR.setPower(v2*trottle);
        motorBL.setPower(v3*trottle);
        motorBR.setPower(v4*trottle);
    }

    //Method to stop all power to the wheel motors
    public void StopWheels() {
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        try { //Pause for amoment to let motion come to a stop
            Thread.sleep(500);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }

    //Method for autonomous driving forward and backwards
    //distance is specifed in inches (positive = drive forward, negative = drive backward)
    //timeout value is used to interrupt drive routine if robot gets stuck
    //and cannot reach the specified destination
    //This method returns true of false to indicate if the robot successfully reached the target
    public boolean Drive(LinearOpMode op, double power, double distance, double timeout){
        //Reset encoder values so target position is relative to the position the robot
        //was at when the method was called
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime drivetime = new ElapsedTime();
        boolean successfulness = false;
        double scaleFactor = 83.33; //Convert inches to encoder values
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
        drivetime.reset(); //start timeout timer
        while (motorBL.isBusy() && op.opModeIsActive() && drivetime.seconds() < timeout){
            //Reduce drive power as robot nears the target to improve accuracy
            if (Math.abs(motorBL.getCurrentPosition() - endPosition) < 500) {
                //0.05 prevents drivePower reducing so much that wheels won't turn
                //and never reach their target positions
                drivePower = 0.05 + power * Math.abs(motorBL.getCurrentPosition() - endPosition)/500 ;
            }
            motorBL.setPower(drivePower);
            motorBR.setPower(drivePower);
            //Have front wheels copy what back wheels are doing
            //in RUN_TO_POSITION mode, motor power is always indicated as positive so
            //use sign of distance to tell front wheels which direction to turn
            motorFL.setPower(Math.signum(distance)*motorBL.getPower());
            motorFR.setPower(Math.signum(distance)*motorBR.getPower());
            op.telemetry.addData("Start", startPosition);
            op.telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
            op.telemetry.addData("Go to", endPosition);
            op.telemetry.addData("BR Position", motorBR.getCurrentPosition());
            op.telemetry.addData("BL Position", motorBL.getCurrentPosition());
            op.telemetry.update();
        }
        //if while loop completed before the timeout, then run was successful
        if (drivetime.seconds() < timeout){
            successfulness = true;
        } else {
            successfulness = false;
        }
        StopWheels(); //stop robot's motion before next method is called
        op.telemetry.addData("Start", startPosition);
        op.telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
        op.telemetry.addData("Go to", endPosition);
        op.telemetry.addData("BR Position", motorBR.getCurrentPosition());
        op.telemetry.addData("BL Position", motorBL.getCurrentPosition());
        op.telemetry.update();

        return successfulness;
    }

    //Method for autonomous driving right
    //Needs more testing, do not use
    public void Right(LinearOpMode op, double power, double distance) {
        //Drive distance in inches. Use "scaleFactor" to convert inches to encoder values.
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

    //Method for autonomous driving left
    //Needs more testing, do not use
    public void Left(LinearOpMode op, double power, double distance) {
        //Drive distance in inches. Use "scaleFactor" to convert inches to encoder values.
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

    //Method for autonomous turning
    // + is left (CCW), - is right (CW)
    public void Turn(LinearOpMode op, double Angle){
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double targetAngle;
        double turnPower = 0.4;
        double threshold = .1;
        double initalDiff;
        double difference;
        //Correct for gyro's sign switch at 180
        if (Math.abs(initialAngle + Angle) > 180) {
            targetAngle = initialAngle + Angle - Math.signum(initialAngle + Angle)*360;
        } else {
            targetAngle = initialAngle + Angle;
        }
        initalDiff = targetAngle - imu.getAngularOrientation().firstAngle;
        difference = initalDiff;
        //Base turnPower off remaining turn angle
        while (Math.abs(difference) > threshold && op.opModeIsActive()) {
            turnPower = Math.signum(difference)*Math.sqrt(Math.abs(difference/initalDiff))/2;
            if (Math.abs(difference) <= 180) { //Turn in the direction minimizes rhe turn angle
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



}
