package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode {
    //    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors
    Thunderbot_2021 robot = new Thunderbot_2021();
    HardwareMap hwMap = null;

    double position2 = 0;
    double position3 = 0;
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Init", "Start");
        telemetry.addData("Init", "Done");

    }

    public void start() {
        telemetry.addData("Starting", "...");
    }

    @Override
    public void loop() {
        robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("linearSlide Pos: ", robot.linear.linearSlide.getCurrentPosition()/robot.linear.COUNTS_PER_CM);

        telemetry.addData("Basket servo Pos:", robot.linear.basketServo.getPosition());
        telemetry.addData("Right servo Pos:", robot.linear.linearSlideServoR.getPosition());
        telemetry.addData("Left servo Pos:", robot.linear.linearSlideServoL.getPosition());

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        //LINEAR SLIDE
        if (gamepad2.b) {
            robot.linear.extend();
            if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                robot.linear.basketMove(0.38);
            }
        } else if(gamepad2.a) {
            robot.linear.reverse();
            if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                robot.linear.basketMove(0.48);
            }
        } else {
            robot.linear.stopExtend();
        }

        //INTAKE
        if (gamepad2.x) {
            robot.intake.intakeMove(-0.9);
        } else if (gamepad2.y) {
            robot.intake.intakeMove(0.9);
        } else {
           robot.intake.intakeStop();
            }

        //LINEAR SLIDE SERVOS
        position2 = robot.linear.linearSlideServoR.getPosition();

        if (gamepad2.dpad_down) {
            position2 = position2 + 0.01;
            position2 = Range.clip(position2, 0, 1);
            robot.linear.servoTurn(position2);
        } else if (gamepad2.dpad_up) {
            position2 = position2 - 0.01;
            position2 = Range.clip(position2, 0, 1);
            robot.linear.servoTurn(position2);
        }

        telemetry.addData("position2: ", position2);

        position3 = robot.linear.basketServo.getPosition();

        if (gamepad2.left_bumper) {
            position3 = position3 + 0.3;
            position3 = Range.clip(position3, 0, .3);
            robot.linear.basketMove(position3);
        } else if (gamepad2.right_bumper){
            position3 = position3 + 0.2;
            position3 = Range.clip(position3, 0, .35);
            robot.linear.basketMove(position3);
        }

        // PRE-SETS
        if (gamepad2.dpad_left) {
            robot.linear.holding();
        } else if(gamepad2.dpad_right){
            robot.linear.dropping();
        }

        //CAROUSEL
        if(gamepad1.right_bumper) {
            robot.carousel.spin(0.65);
        } else if(gamepad1.left_bumper) {
            robot.carousel.spin(-0.65);
        } else {
            robot.carousel.spinStop();
        }
    }
}