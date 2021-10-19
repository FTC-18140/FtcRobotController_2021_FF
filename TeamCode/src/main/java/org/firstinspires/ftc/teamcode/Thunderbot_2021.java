package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Thunderbot_2021 {
    /**
     * Public OpMode members
     */
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    BNO055IMU imu = null;


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

    double gyStartAngle = 0;
    double initialPosition = 0;
    boolean moving = false;
    public boolean drive (double direction, double distance, double power) {

        double xValue = Math.sin(toRadians(direction)) * power;
        double yValue = Math.cos(toRadians(direction)) * power;
        double currentAngle = updateHeading();
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        if(!moving){
            gyStartAngle = updateHeading();
            initialPosition = leftFront.getCurrentPosition();
            moving = true;
        }

        double position = abs(leftFront.getCurrentPosition() - initialPosition);
        double positionInCM = position/COUNTS_PER_CM;

        if(positionInCM >= distance){
            stop();
            moving = false;
            return true;

        } else if (currentAngle != gyStartAngle) { // Could adjust if not precise enough

            // calculates required speed to adjust to gyStartAngle
            leftFrontSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightFrontSpeed = power - (currentAngle - gyStartAngle) / 100;
            leftRearSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightRearSpeed = power - (currentAngle - gyStartAngle) / 100;

            // Setting range of adjustments (I may be wrong about this)
            leftFrontSpeed = Range.clip(leftFrontSpeed, -1, 1);
            rightFrontSpeed = Range.clip(rightFrontSpeed, -1, 1);
            leftRearSpeed = Range.clip(leftRearSpeed, -1, 1);
            rightRearSpeed = Range.clip(rightRearSpeed, -1, 1);

            // Set new targets
            leftFront.setPower(leftFrontSpeed);
            leftRear.setPower(leftRearSpeed);
            rightFront.setPower(rightFrontSpeed);
            rightRear.setPower(rightRearSpeed);
            return false;


        } else {
            joystickDrive(yValue, xValue, 0);
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
