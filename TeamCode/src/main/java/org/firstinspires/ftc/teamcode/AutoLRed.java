package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="RedWarehouse", group="Auto")
public class AutoLRed extends OpMode {
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
                    done = robot.drive(180, 33, 0.25);
                } else {
                    resetStartTime();
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done) {
                    done = robot.linear.extendSlide(70, 0.5);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (getRuntime() < 1) {
                    robot.linear.servoTurn(0.2);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 3:
                if (getRuntime() < 1) {
                    robot.linear.basketMove(0.6);
                } else {
                    done = false;
                    state++;
                }
                break;

            case 4:
                if (!done) {
                    done = robot.linear.retractSlide(70  , 0.15);
                    robot.linear.servoTurn(1);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (!done) {
                    done = robot.turnTo(80, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 6:
                if (!done) {
                    done = robot.drive(180, 170, 0.75);
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

