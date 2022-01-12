package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.tensorflow.lite.task.core.vision.ImageProcessingOptions;


public class Thunderbot_2021 {
    /**
     * Public OpMode members
     */
    BNO055IMU imu = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    // pulling in the other areas
    LinearSlide linear = new LinearSlide();
    CarouselRyan carousel = new CarouselRyan();
    armMotor arm = new armMotor();
    intake intake = new intake();
   // DcMotor armMotor = null;


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
     * Initialize standard Hardware interfaces
     */
    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
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
            imu = ahwMap.get(BNO055IMU.class, "imu 1");
            imu.initialize(parameters);
        } catch (Exception p_exeception) {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        hwMap = ahwMap;
        telemetry = telem;

        // Define & Initialize Motors
        rightFront = hwMap.dcMotor.get("rf");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hwMap.dcMotor.get("rr");
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hwMap.dcMotor.get("lf");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftRear = hwMap.dcMotor.get("lr");
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear.init(hwMap, telemetry);

        carousel.init(hwMap, telemetry);

        arm.init(hwMap, telemetry);

        intake.init(hwMap, telemetry);
    }
    public void joystickDrive (double foward, double right, double clockwise){
        double frontLeft = foward + clockwise + right;
        double frontRight = foward - clockwise - right;
        double backLeft = foward + clockwise - right;
        double backRight = foward - clockwise + right;

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
    double initial = 0;
    double initialPosition = 0;
    boolean moving = false;
    public boolean drive (double direction, double distance, double power) {

        double xValue = Math.sin(toRadians(direction)) * power;
        double yValue = Math.cos(toRadians(direction)) * power;


        if (!moving) {
            initialPosition = leftFront.getCurrentPosition();
            moving = true;
        }

        double position = abs(leftFront.getCurrentPosition() - initialPosition);
        double positionInCM = position / COUNTS_PER_CM;

        if (positionInCM >= distance) {
            stop();
            moving = false;
            return true;

        } else {
            joystickDrive(yValue, xValue, 0);
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
