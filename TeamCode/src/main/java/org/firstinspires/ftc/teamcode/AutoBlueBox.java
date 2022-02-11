package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="BlueStorage", group="Auto")

public class AutoBlueBox extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Init Done:", "yes");
    }

    public void start() {
        telemetry.addData("Start:", "pressed");
    }

    int state = 0;
    boolean done = false;

    @Override
    public void loop () {
        switch (state) {
            case 0:
                if (!done) {
                    done = robot.drive(245, 10, 0.75);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done) {
                    done = robot.turn(50, 0.25);
                } else {
                    resetStartTime();
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (getRuntime() < 3) {
                    robot.carousel.autoSpin(-0.6);
                } else {
                    robot.carousel.spinStop();
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (!done) {
                    done = robot.turn(30, 0.25);
                } else {
                    resetStartTime();
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done) {
                    done = robot.drive(90, 48, 0.6);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (!done) {
                    done = robot.drive(180, 6, 0.6);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            default:
                break;
        }
    }

}



