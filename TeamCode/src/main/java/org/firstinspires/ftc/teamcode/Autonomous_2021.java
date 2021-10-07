package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Autonomous_2021", group="Autonomous")
public class Autonomous_2021 extends OpMode{

    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init(){}

    public void start(){}

    public void loop() {

        robot.driveForward(0.3,61);
    }
}