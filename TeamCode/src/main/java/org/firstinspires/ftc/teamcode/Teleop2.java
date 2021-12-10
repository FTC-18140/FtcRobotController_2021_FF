package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

@TeleOp(name="Teleop2", group="Teleop")
public class  Teleop2 extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
    robot.init(hardwareMap,telemetry);
    }

    public void start(){

    }

    public void loop() {
    robot.joystickDrive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

        telemetry.addData("lx: ", gamepad1.left_stick_x);
        telemetry.addData("ly: ", gamepad1.left_stick_y);
        telemetry.addData("rx: ", gamepad1.right_stick_x);
        telemetry.addData("ry: ", gamepad1.right_stick_y);

        telemetry.update();

        //the right carousel spins clockwise
        if (gamepad1.dpad_left){
            robot.carousel.spin(0.5);
        }
    }


}