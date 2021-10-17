package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Thunderbot_2021 {

    /**
     * Public OpMode members
     */
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / ((WHEEL_DIAMETER_INCHES * 2.54) * 3.1415);

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
    public void joystickDrive(double forward, double right, double clockwise) {
        double mfrontRight = forward + clockwise + right;
        double mfrontLeft = forward - clockwise - right;
        double mbackLeft = forward + clockwise - right;
        double mbackRight = forward - clockwise + right;

        double max = abs(mfrontLeft);
        if (abs(mfrontRight) > max) {
            max = abs(mfrontRight);
        }
        if (abs(mbackLeft) > max) {
            max = abs(mbackLeft);
        }

        if (abs(mbackRight) > max) {
            max = abs(mbackRight);
        }
       if (max > 1) {
            mfrontLeft /= max;
            mfrontRight /= max;
            mbackLeft /= max;
            mbackRight /= max;
        }

       rightFront.setPower(-mfrontRight);
       leftFront.setPower(-mfrontLeft);
       rightRear.setPower(-mbackRight);
       leftRear.setPower(-mbackLeft);

    }

    double initialPos = 0;
    boolean moving = false;
    public boolean drive(double distance, double power) {
        if(!moving) {
            initialPos = leftFront.getCurrentPosition();
            moving = true;
        }
        double currentPosition = leftFront.getCurrentPosition();

        double howFar = currentPosition - initialPos;
        double howFarCM = howFar / COUNTS_PER_CM;
        if (distance <= howFarCM) {
            stop();
            moving = false;
            return true;

        }
        else {
            joystickDrive(-power, 0, 0);
            return false;
        }
    }
    // Stop all motors
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        }
    }


