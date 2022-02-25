package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="DroppingFreightMid", group="Auto")
public class DroppingFreightMid extends OpMode {
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
    public void loop() {
        telemetry.addData("state ", state);
        switch (state) {
            case 0:
                if (!done) {
                    done = robot.linear.extendSlide(25, 0.2);
                } else {
                    resetStartTime();
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;

            case 1:
                if (getRuntime() < 3) {
                    robot.linear.servoTurn(0);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 2:
                if (getRuntime() < 1) {
                    robot.linear.basketMove(0.2);
                } else {
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (getRuntime() < 2) {
                    robot.linear.servoTurn(0.95);
                } else {
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done) {
                    done = robot.linear.retractSlide(24, 0.2);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;

            default:
                break;
        }
    }
}