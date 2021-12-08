package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

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
    DcMotor linearSlide = null;
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

        linearSlide = hwMap.dcMotor.get("linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*armMotor = hwMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    */
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


        if(!moving){
            initialPosition = leftFront.getCurrentPosition();
            moving = true;
        }

        double position = abs(leftFront.getCurrentPosition() - initialPosition);
        double positionInCM = position/COUNTS_PER_CM;

        if(positionInCM >= distance) {
            stop();
            moving = false;
            return true;

        } else {
            joystickDrive(yValue, xValue, 0);
            return false;
        }
    }

    public boolean linear(double distance, double power) {
        
        if (!moving) {
            initial = linearSlide.getCurrentPosition();

            moving = true;
        }
       double position2 = abs(linearSlide.getCurrentPosition() - initial);
        double positionInCM2 = position2 / COUNTS_PER_CM;

        if (positionInCM2 >= distance) {
            stop();
            moving = false;

        } else {
            linearSlide.setPower(power);
        }
        return true;

    }
    /*
    public void arm(double distance, double power) {

        if(!moving) {
            initial = armMotor.getCurrentPosition();

            moving = true;
        }
        double position3 = abs(armMotor.getCurrentPosition() - initial);
        double positionInCM3 = position3 / COUNTS_PER_CM;

        if (positionInCM3 >= distance) {
            stop();
            moving = false;
        } else {
            armMotor.setPower(power);
        }
    }
*/

    // Stop all motors
    public void stop() {
      //  linearSlide.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
