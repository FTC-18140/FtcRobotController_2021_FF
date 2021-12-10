package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    DcMotor linearSlideR = null;
    DcMotor linearSlideL = null;
    HardwareMap hwMap = null;

    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    private Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry telem) {
        hwMap = ahwMap;
        telemetry = telem;

        linearSlideR = hwMap.dcMotor.get("linearL");
        linearSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideL = hwMap.dcMotor.get("linearR");
        linearSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    double initial = 0;
    boolean moving = false;

    public void extend(double distance, double power) {
        linearSlideL.setPower(power);
        linearSlideR.setPower(power);

        if (!moving) {
            initial = linearSlideR.getCurrentPosition();

            moving = true;
        }
        double position3 = abs(linearSlideR.getCurrentPosition() - initial);
        double positionInCM3 = position3 / COUNTS_PER_CM;

        if (positionInCM3 >= distance) {
            stopExtend();
            moving = false;

        }
    }
        public void stopExtend () {
            linearSlideL.setPower(0);
            linearSlideR.setPower(0);
        }
    }
