package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode {
    //    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors
    Thunderbot_2021 robot = new Thunderbot_2021();

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

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        // above is the code for the basic motor

        robot.linear.extend(100, gamepad1.right_trigger);
        robot.linear.extend(-100, -gamepad1.left_trigger);

        if (gamepad1.x) {
            robot.intake.intakeMove();
        } else {
            robot.intake.intakeStop();
        }
        if(gamepad1.dpad_up) {
            robot.linear.servoTurn();
        } else if (gamepad1.dpad_down) {
            robot.linear.negativeServoTurn();
        } else {
            robot.linear.servoEase();
        }
        //  above is the code that makes the linear slide extend and retract

        if (gamepad1.left_bumper) {
            robot.arm.close();
        } else {
            robot.arm.open();
        }

        // above is the code for the arm's claws closing whenever you press and hold the left bumper

        if (gamepad1.right_bumper) {
            robot.carousel.spin(-1);
        } else {
            robot.carousel.spinStop();
        }

        // above is the code for the carousel spinning whenever you press the right bumper

        if (gamepad1.a) {
            robot.arm.lift(-1);
        } else if (gamepad1.b) {
            robot.arm.lift(1);
        } else {
            robot.arm.stopLift();
        }

        // above is the code that moves the arm up and down whenever you press a and b

    }
}

