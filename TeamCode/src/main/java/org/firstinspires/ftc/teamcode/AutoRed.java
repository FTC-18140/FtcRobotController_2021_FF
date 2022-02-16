package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="RedCarousel", group="Auto")
public class AutoRed extends OpMode {
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
                    done = robot.drive(135, 20, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done) {
                    done = robot.turnTo(-160, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (!done) {
                    done = robot.drive(-90, 10, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (!done) {
                    done = robot.drive(150, 9, 0.25);; //robot.turnTo(150, 0.25);
                } else {
                    resetStartTime();
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (getRuntime() < 3) {
                    robot.carousel.autoSpin(0.6);
                } else {
                    robot.carousel.spinStop();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if(!done) {
                    done = robot.drive(45,45, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 6:
                if (!done) {
                    done = robot.turnTo(95, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 7:
                if (!done) {
                    done = robot.drive(0, 82, 0.75);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 8:
                if (!done) {
                    done = robot.turnTo(3, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 9:
                if (!done) {
                    done = robot.drive(180, 4, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 10:
                if (!done) {
                    done = robot.linear.extendPosition(85, 0.25);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 11:
                if (getRuntime() < 3) {
                    robot.linear.servoTurn(-0.5);
                } else {
                    robot.linear.servoTurn(0);
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 12:
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
            case 13:
                if (!done) {
                    done = robot.turnTo(-80, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 14:
                if (!done) {
                    done = robot.drive(180, 130, 1);
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
