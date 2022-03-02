package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="freightTest", group="Auto")
public class PlacingFreightTest extends OpMode {
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
        telemetry.addData("state", state);
        switch (state) {
            case 0:
                if (!done) {
                    done = robot.linear.extendSlide(45, 0.5);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (getRuntime() < 1) {
                    robot.linear.servoTurn(0);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 2:
                if (!done) {
                    done = robot.linear.retractSlide(45, 0.5);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (!done) {
                    done = robot.drive(180, 8, 0.1);
                } else {
                    robot.stop();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (getRuntime() < 1) {
                    robot.linear.basketMove(0.25);
                } else {
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (!done) {
                    done = robot.drive(0, 16, 0.1);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 6:
                if (!done) {
                    done = robot.linear.extendSlide(45, 0.5);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 7:
                if (getRuntime() < 1.5) {
                    robot.linear.servoTurn(1);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;

            case 8:
                if (!done) {
                    done = robot.linear.retractSlide(45  , 0.5);
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