package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public void start() { telemetry.addData("Starting", "..."); }

    @Override
    public void loop() {
        robot.joystickDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);
        // above is the code for the basic motors
        robot.linear.extend(20, -gamepad1.right_trigger);
        robot.linear.extend(0, gamepad1.left_trigger);
        // code for the linear slide
        telemetry.addData("LinearP", gamepad1.right_trigger);
        telemetry.addData("LinearN", -gamepad1.left_trigger);
        if (gamepad1.right_bumper) {
            robot.carousel.spin(1);
        } else {
            robot.carousel.spinStop();
        }


        // carousels code
       /* // making the code for the arm
        if(gamepad1.a = true) {
            robot.armMotor.setPower(0.5);
        } else if (gamepad1.b = true) {
            robot.armMotor.setPower(-0.5);
        } else {
            stop();
        }
        telemetry.update();
        */

    }

    }

