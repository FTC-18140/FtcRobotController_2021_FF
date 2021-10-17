package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop3", group="Teleop")
public class TeleOp3 extends OpMode {

    Thunderbot_2021 robot = new Thunderbot_2021();


    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("init = ", "starting");
        telemetry.addData("init = ", "done");
    }

    public void start() {
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_x;
        double clockwise = gamepad1.left_stick_x;

        robot.joystickDrive(forward, right, clockwise);

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        telemetry.update();
    }

}

