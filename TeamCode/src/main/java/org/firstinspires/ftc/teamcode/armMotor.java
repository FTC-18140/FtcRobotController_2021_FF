package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class armMotor {
    DcMotor armMotor = null;
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

        armMotor = hwMap.dcMotor.get("arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    boolean moving = false;
    double initial = 0;
        public void lift(double distance, double power) {
            armMotor.setPower(power);

            if (!moving) {
                initial = armMotor.getCurrentPosition();

                moving = true;
            }
            double position3 = abs(armMotor.getCurrentPosition() - initial);
            double positionInCM3 = position3 / COUNTS_PER_CM;

            if (positionInCM3 >= distance) {
            stopLift();
            }
        }
        public void stopLift() {
            armMotor.setPower(0);
        }
        public void close() {

        }
    }

