package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Thunderbot_2021 {
    /**
     * Public OpMode members
     */
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    BNO055IMU imu = null;

    Vision theCamera = null;

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    /**
     * local OpMode members
     */
    HardwareMap hwMap = null;
    private Telemetry telemetry;

    /**
     * Constructor
     */
    public Thunderbot_2021() {

    }

    /**
     * Initializes the robot
     * @param ahwMap -
     * @param telem -
     */
    public void init (HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;

        try {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU.
            imu = ahwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (Exception p_exeception) {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Define & Initialize Motors
        rightFront = hwMap.dcMotor.get("rf");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hwMap.dcMotor.get("rr");
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hwMap.dcMotor.get("lf");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hwMap.dcMotor.get("lr");
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        theCamera.init(ahwMap, telem);
    }

    /**
     * Uses Stick Values To Move the Robot
     * @param forward - Y-Axis Value On The Left Stick
     * @param right - X-Axis Value On The Left Stick
     * @param clockwise - X-Axis Value On The Right Stick
     */
    public void joystickDrive (double forward, double right, double clockwise){
        double frontLeft = forward + clockwise + right;
        double frontRight = forward - clockwise - right;
        double backLeft = forward + clockwise - right;
        double backRight = forward - clockwise + right;

        double max = abs(frontLeft);
        if (abs(frontRight) > max){
            max = abs(frontRight);
        }
        if (abs(backLeft) > max){
            max = abs(backLeft);
        }
        if (abs(backRight) > max){
            max = abs(backRight);
        }
        if (max > 1) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        leftFront.setPower(frontLeft);
        rightFront.setPower(frontRight);
        leftRear.setPower(backLeft);
        rightRear.setPower(backRight);
    }


    double gyStartAngle = 0;
    double initialPosition = 0;
    boolean moving = false;

    /**
     * Drives in a specified direction for a specified distance
     * @param direction - Direction in Degrees The Robot Will Drive
     * @param distance - Distance The Robot Will Travel
     * @param power - Speed The Robot Will Travel
     * @return
     */
    public boolean drive (double direction, double distance, double power) {

        // can it go diagonal left
        // 360 or 180 -180
   
        double xValue = Math.sin(toRadians(direction)) * power;
        double yValue = Math.cos(toRadians(direction)) * power;
        double currentAngle = updateHeading();
        telemetry.addData("current angle", updateHeading());
        telemetry.update();

        // Set initial angle and position
        if(!moving){
            gyStartAngle = updateHeading();
            initialPosition = leftFront.getCurrentPosition();
            moving = true;
        }

        double position = abs(leftFront.getCurrentPosition() - initialPosition);
        double positionInCM = position/COUNTS_PER_CM;

            // calculates required speed to adjust to gyStartAngle
            double adjust = (currentAngle - gyStartAngle) / 100;
            // Setting range of adjustments
            adjust = Range.clip(adjust, -1, 1);

        // Stops when at the specified distance
        if(positionInCM >= distance){
            stop();
            moving = false;
            return true;

            // Continues if not at the specified distance
        } else {
            joystickDrive(yValue, xValue, -adjust);
            return false;
        }
    }


    double startAngle = 0;
    double currentAngle = 0;

    /**
     * Turns an Exact Angle in Degrees Using The Gyro
     * @param degrees - Angle The Robot Will Turn
     * @param power - Speed The Robot will Turn
     * @return
     */
    public boolean turn (double degrees, double power){
        power = abs(power);
        // Sets initial angle
        if(!moving){
            currentAngle = updateHeading();
            startAngle = currentAngle;
            moving = true;
        }

        // Updates current angle
        currentAngle = updateHeading();
        telemetry.addData("current angle", updateHeading());
        telemetry.update();

        if (0 > degrees){
            power = -power;
        }

        // what happens if above 180

        if (abs(degrees) == 180){
            // avoid 180 somehow
        }

        // Stops turning when at the specified angle
        if(Math.abs(currentAngle - startAngle) >= abs(degrees)){
                stop();
                moving = false;
                return true;

        // Continues to turn if not at the specified angle
        }else{
            joystickDrive(0, 0, power);
            return false;
        }
    }

    /**
     * OnShape measurement
     * 
     * metal piece
     * w - 40.7mm
     * l - 47.3mm
     * thicc - 2.2mm
     *
     * cam base
     * top w - 40.2mm
     * bot w - 43.4
     * l - 52.5
     * thicc - 10.4
     */

    public int getBarcode()
    {
        return theCamera.getBarcode();
    }

    // Gets the current angle of the robot
    public double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }


    // Stop all motors
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
