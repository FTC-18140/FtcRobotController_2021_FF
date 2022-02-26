package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class LinearSlide {
    DcMotor linearSlide = null;
    DcMotor linearSlideP = null;
    Servo linearSlideServoL = null;
    Servo linearSlideServoR = null;
    Servo basketServo = null;
    HardwareMap hwMap = null;

    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 12;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    private Telemetry telemetry;
    private int state;
    boolean done = false;
    private long startTime = 0; // in nanoseconds

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

        basketServo = hwMap.servo.get("bs");
        basketServo.setDirection(REVERSE);

        linearSlideServoL = hwMap.servo.get("lssl");
        linearSlideServoR = hwMap.servo.get("lssr");
        linearSlideServoR.setDirection(REVERSE);

        linearSlideServoL.setPosition(1);
        linearSlideServoR.setPosition(1);
        basketServo.setPosition(0.38);
    }

    double initial = 0;
    boolean moving = false;
    public boolean retractSlide(double distance, double power) {
        if (!moving) {
            initial = linearSlide.getCurrentPosition();
            moving = true;
        }

        double position = abs(linearSlide.getCurrentPosition() - initial);
        double positionInCM = position / COUNTS_PER_CM;

        if (positionInCM >= distance) {
            stopExtend();
            moving = false;
            return true;
        } else {
            linearSlide.setPower(power);
            return false;
        }
    }

    public boolean extendSlide(double distance, double power) {
        if (!moving) {
            initial = linearSlide.getCurrentPosition();
            moving = true;
        }

        double position = abs(linearSlide.getCurrentPosition() - initial);
        double positionInCM = position / COUNTS_PER_CM;

        telemetry.addData("Slide Position", positionInCM);
        if (-positionInCM <= -distance) {
            stopExtend();
            moving = false;
            return true;
        } else {
            linearSlide.setPower(-power);
            return false;
        }
    }

        public void stopExtend () {
            linearSlide.setPower(0);
        }
        public void extend() {
            linearSlide.setPower(1);
        }
        public void reverse() {
            linearSlide.setPower(-1);
        }
        public void servoTurn(double position) {
        linearSlideServoL.setPosition(position);
        linearSlideServoR.setPosition(position);
        basketServo.setPosition(position * 0.4);

        telemetry.addData("Position in servoTurn: ", linearSlideServoR.getPosition());
    }
    public void basketMove(double position) {
        basketServo.setPosition(position);
        telemetry.addData("wrist servo", basketServo.getPosition());
    }

    public void holding (){
        servoTurn(0.18);
    }

    public boolean dropping () {
        boolean allDone = false;
        switch (state) {
            case 0:
                if (getRuntime() < 2) {
                    servoTurn(0.2);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 1:
                if (getRuntime() < 1) {
                    basketMove(0.6);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 2:
                if (getRuntime() < 1) {
                    servoTurn(0.95);
                } else {
                    done = false;
                    allDone = true;
                    state++;
                }
                break;

            default:
                break;
        }
        return allDone;
    }
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetStartTime() {
        startTime = System.nanoTime();
    }
}
