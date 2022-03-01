package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Thunderbot_2021;

@Autonomous(name="CV", group="Auto")
public class CVauto extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();
    Vision vision = new Vision();
    public void init() {
        robot.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);
        telemetry.addData("Init Done:", "yes");
    }

    public void start() {
        telemetry.addData("Start:", "pressed");
    }

    int state = 0;
    boolean done = false;

    @Override
    public void loop() {
        telemetry.addData("state", state);
        switch (state) {
            case 0:

        }
    }
}