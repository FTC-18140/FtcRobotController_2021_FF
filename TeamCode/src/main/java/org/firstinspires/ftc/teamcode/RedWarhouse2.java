package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="RedWarehouse2", group="Auto")
public class RedWarhouse2 extends OpMode {
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
                    done = robot.turn(-20, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 1: //Variable
                if (!done) {
                    done = robot.linear.extendSlide(45, 0.5);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (getRuntime() < 1) {
                    robot.linear.servoTurn(0);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 3:
                if (!done) {
                    done = robot.linear.retractSlide(45, 0.5);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done) {
                    done = robot.drive(180, 50, 0.2);
                } else {
                    robot.stop();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (getRuntime() < 1) {
                    robot.linear.basketMove(0.25);
                } else {
                    done = false;
                    state++;
                }
                break;
            case 6:
                if (!done) {
                    done = robot.drive(0, 16, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 7:
                if (!done) {
                    done = robot.linear.extendSlide(45, 0.5);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 8:
                if (getRuntime() < 1.5) {
                    robot.linear.servoTurn(1);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;

            case 9:
                if (!done) {
                    done = robot.linear.retractSlide(45  , 0.5);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 10:
                if (!done) {
                    done = robot.turnTo(70, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 11:
                if (!done) {
                    done = robot.drive(180, 150, 0.75);
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
