package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="AutoRyan", group="Autonomous")
public class AutoRyan extends OpMode {

    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("init = ", "starting");
        telemetry.addData("init = ", "done");
    }
    public void start() {
    }
    boolean second = false;
    @Override
    public void loop() {
        if (robot.moving = true) {
            robot.drive(60, 0.5);
            second = true;
        }
            if (second = true) {
                robot.drive(-60, -0.5);
            } else {
                stop();
        }
            }
        }

