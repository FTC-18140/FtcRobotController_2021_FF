package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode {
    //    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors
    Thunderbot_2021 robot = new Thunderbot_2021();
    HardwareMap hwMap = null;

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
        //ROBOT
        telemetry.addData("linearSlide Pos: ", robot.linear.linearSlide.getCurrentPosition());
        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        // above is the code for the basic motor

        //LINEAR SLIDE
        if (gamepad2.a) {
            robot.linear.extend();
           } else if(gamepad2.b) {
            robot.linear.reverse();
        } else {
            robot.linear.stopExtend();
        }

        if (gamepad2.right_bumper) {
            robot.linear.linearSlideP.setPower(-1);
        } else if (gamepad2.left_bumper) {
            robot.linear.linearSlideP.setPower(1);
        } else {
            robot.linear.linearSlideP.setPower(0);
        }




        //INTAKE
        if (gamepad2.x) {
            robot.intake.intakeMove(-0.9);
     //       robot.linear.extendPosition(0, 0.5);
        } else if (gamepad2.y) {
            robot.intake.intakeMove(0.9);
       //     robot.linear.extendPosition(5, 0.5);
        } else {
           robot.intake.intakeStop();
         //  robot.linear.stopExtend();
            }
        //LINEAR SLIDE SERVOS
        robot.linear.servoTurn(gamepad2.left_stick_y);


    //  above is the code that makes the linear slide extend and retract
    // above is the code for the arm's claws closing whenever you press and hold the left bumper
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