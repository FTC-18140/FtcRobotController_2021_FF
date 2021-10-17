package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;



import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode {
//    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor intake;
    DcMotor shooterMotor2;
    DcMotor shooterMotor;
    DcMotor armMotor;

    //servos

    Servo leftClaw;
    Servo rightClaw;
    CRServo intakeServo;
    CRServo intakeServoTwo;
    CRServo rampIntakeServo;
    CRServo shooterServo1;
    CRServo shooterServo2;

    //touch sensors

    TouchSensor touchSensor1;
    TouchSensor touchSensor2;

    //digital channels

    DigitalChannel digitalTouch;
    DigitalChannel digitalTouch2;


    public void init() {
        telemetry.addData("Init started", "");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftRear = hardwareMap.get(DcMotor.class, "lr");
        rightRear = hardwareMap.get(DcMotor.class, "rr");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("", "init ended");
    }

    public void start() {


        }
    @Override
    public void loop() {
        joystickDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

//        telemetry.update();
    }


    public void joystickDrive(double forward, double right, double clockwise) {
        double mfrontLeft = forward + clockwise + right;
        double mfrontRight = forward - clockwise - right;
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

        leftFront.setPower(mfrontLeft);
        rightFront.setPower(mfrontRight);
        leftRear.setPower(mbackLeft);
        rightRear.setPower(mbackRight);
    }

    // Stop all motors
    boolean moving = false;
    public void drive(double distance, double power) {
        stop();
    }
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
