package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    DcMotor linearSlide = null;
    DcMotor linearSlideP = null;
    CRServo linearSlideServoL = null;
    CRServo linearSlideServoR = null;
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

        linearSlideP = hwMap.dcMotor.get("linearPull");
        linearSlideP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideP.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        linearSlideServoL = hwMap.crservo.get("lssl");
        linearSlideServoR = hwMap.crservo.get("lssr");
    }

    double initial = 0;
    boolean moving = false;

    public boolean extend(double distance, double power) {
        linearSlide.setPower(power);

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
        public void servoTurn(double power) {
            linearSlideServoL.setPower(power);
            linearSlideServoR.setPower(-power);
        }
        public void negativeServoTurn(double power
        ) {
            linearSlideServoL.setPower(power);
            linearSlideServoR.setPower(-power);
        }
        public void servoHold() {
            linearSlideServoL.setPower(0);
            linearSlideServoR.setPower(0);
        }
        public void bothExtend() {
            linearSlide.setPower(0.75);
            linearSlideP.setPower(-0.75);
        }
    }
