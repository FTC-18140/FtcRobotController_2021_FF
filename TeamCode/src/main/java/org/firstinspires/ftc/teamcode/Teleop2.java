package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode {
//    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors
    Thunderbot_2021 robot = new Thunderbot_2021();
    public void init() {
        telemetry.addData("Init", "Start");
        telemetry.addData("Init", "Done");
    }
    public void start() {


        }
    @Override
    public void loop() {
        robot.joystickDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        robot.linearSlide.setPower(gamepad1.right_trigger);

        telemetry.addData("LinearP", gamepad1.right_trigger);
        telemetry.addData("LinearN", -gamepad1.left_trigger);

//        telemetry.update();
    }

    }

