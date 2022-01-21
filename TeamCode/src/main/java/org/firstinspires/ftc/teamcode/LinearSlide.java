package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    DcMotor linearSlide = null;
    Servo linearSlideServoL = null;
    Servo linearSlideServoR = null;
    HardwareMap hwMap = null;

    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 12;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    private Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry telem) {
        hwMap = ahwMap;
        telemetry = telem;

        linearSlide = hwMap.dcMotor.get("linear");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideServoL = hwMap.servo.get("lssl");
        linearSlideServoR = hwMap.servo.get("lssr");
    }

    double initial = 0;
    boolean moving = false;

    public boolean extend(double distance, double power) {
        linearSlide.setPower(power);

        if(distance < 10) {
            servoLevel();
        } else {
            servoHold();
        }
        if (!moving) {
            initial = linearSlide.getCurrentPosition();

            moving = true;
        }
        double position3 = abs(linearSlide.getCurrentPosition() - initial);
        double positionInCM3 = position3 / COUNTS_PER_CM;

        if (positionInCM3 >= distance) {
            moving = false;
            return true;
        }
        return false;
    }
        public void stopExtend () {
            linearSlide.setPower(0);
        }
        public void servoTurn() {
            linearSlideServoL.setPosition(-1);
            linearSlideServoR.setPosition(1);
        }
        public void negativeServoTurn() {
            linearSlideServoL.setPosition(-0.75);
            linearSlideServoR.setPosition(-0.75);
    }
        public void servoLevel() {
            linearSlideServoL.setPosition(0.4);
            linearSlideServoR.setPosition(0.4);
        }
        public void servoHold() {
            linearSlideServoL.setPosition(-0.15);
            linearSlideServoR.setPosition(0.15);
        }
        /*public void servoStop(){
        linearSlideServoL.setPosition(0);
        linearSlideServoR.setPosition(0);
        }*/
        public void pressure() {
        linearSlideServoL.setPosition(1);
        linearSlideServoR.setPosition(1);
        }
    }
