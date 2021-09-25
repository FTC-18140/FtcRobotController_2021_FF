package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop2021", group="Teleop")
public class Teleop_2021 extends OpMode{

    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init(){}

    public void start(){}

    public void loop() {

    //sent joystick values to robot object
        // joystick values x and y on left stick; x only on right stick.  All that is needed for mecanum drive
        double forward = gamepad1.left_stick_y;     // push left joystick forward to go forward
        double right = -gamepad1.left_stick_x;        // push left joystick to the right to strafe right
        double clockwise = -gamepad1.right_stick_x;   // push right joystick to the right to rotate clockwise

        //call joyStickDrive method from Thunderbot_2021 class for movement
        robot.joyStickDrive(forward, right, clockwise);

        telemetry.addData("lx: ", gamepad1.left_stick_x);
        telemetry.addData("ly: ", gamepad1.left_stick_y);
        telemetry.addData("rx: ", gamepad1.right_stick_x);
        telemetry.addData("ry: ", gamepad1.right_stick_y);

        telemetry.update();
    }
}