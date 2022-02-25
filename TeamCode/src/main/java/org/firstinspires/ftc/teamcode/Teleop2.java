package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="PrototypeTest", group="Teleop")
public class Teleop2 extends OpMode {
    //    Thunderbot_2021 robot = new Thunderbot_2021();
    //motors
    Thunderbot_2021 robot = new Thunderbot_2021();
    HardwareMap hwMap = null;
    double position2 = 0;
    double position3 = 0 ;
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Init", "Start");
        telemetry.addData("Init", "Done");

        position2 = robot.linear.linearSlideServoL.getPosition();
    }

    public void start() {
        telemetry.addData("Starting", "...");
    }

    @Override
    public void loop() {
        telemetry.addData("linearSlide Pos: ", robot.linear.linearSlide.getCurrentPosition()/robot.linear.COUNTS_PER_CM);

        telemetry.addData("Basket servo Pos:", robot.linear.basketServo.getPosition());
        telemetry.addData("Right servo Pos:", robot.linear.linearSlideServoR.getPosition());
        telemetry.addData("Left servo Pos:", robot.linear.linearSlideServoL.getPosition());

        //LINEAR SLIDE
        if (gamepad1.b) {
            robot.linear.extend();
           } else if(gamepad1.a) {
            robot.linear.reverse();
        } else {
            robot.linear.stopExtend();
        }

        //INTAKE
        if (gamepad1.x) {
            robot.intake.intakeMove(0.9);
        } else if (gamepad1.y) {
            robot.intake.intakeMove(-0.9);
        } else {
           robot.intake.intakeStop();
        }

        //LINEAR SLIDE SERVOS
        if (gamepad1.dpad_up) {
            position2 = position2 + 0.01;
            position2 = Range.clip(position2, 0, 1);
            robot.linear.servoTurn(position2);
        } else if (gamepad1.dpad_down) {
            position2 = position2 - 0.01;
            position2 = Range.clip(position2, 0, 1);
            robot.linear.servoTurn(position2);
        }

        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("position2: ", position2);

        position3 = robot.linear.basketServo.getPosition();

        if (gamepad1.left_bumper) {
            position3 = position3 + 0.3;
            position3 = Range.clip(position3, 0, .3);
            robot.linear.basketMove(position3);
        } else if (gamepad1.right_bumper){
            position3 = position3 - 0.2; 
            position3 = Range.clip(position3, 0, .2);
            robot.linear.basketMove(position3);
        }
    }
}