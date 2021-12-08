package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode {
//    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors
        CRServo rs;
        CRServo ls;
    Thunderbot_2021 robot = new Thunderbot_2021();
    public void init() {
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
       robot.linearSlide.setPower(gamepad1.left_trigger);
       robot.linearSlide.setPower(-gamepad1.left_trigger);
        // code for the linear slide
        telemetry.addData("LinearP", gamepad1.right_trigger);
        telemetry.addData("LinearN", -gamepad1.left_trigger);


        // carousels code
        if(gamepad1.right_bumper = true) {
            rs.setPower(1);
        } else if(gamepad1.right_bumper = false) {
            rs.setPower(0);
        } else if(gamepad1.left_bumper = true) {
            ls.setPower(1);
        } else if (gamepad1.left_bumper = false){
            ls.setPower(0);
        }
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

