package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Disabled
@Autonomous(name="LinearSlideAuto", group="Auto")

public class LinearSlideAuto extends OpMode {
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
                    done = robot.linear.extendPosition(85, 0.25);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (getRuntime() < 3) {
                    robot.linear.servoTurn(-0.5);
                } else {
                    robot.linear.servoTurn(0);
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

                case 2:
                if (!done) {
                    done = robot.linear.extendPosition(50, -0.25);
                    robot.linear.servoTurn(1);
                } else {
                    robot.linear.servoTurn(0);
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



