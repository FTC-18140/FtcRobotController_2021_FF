package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Thunderbot_2021;

@Autonomous(name="RedStorage", group="Auto")
public class RedStorage extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();
    Vision vision = new Vision();
    String duckPosition;

    public void init() {
        robot.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);
        telemetry.addData("Init Done:", "yes");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Duck Position: ", vision.stageSwitchingPipeline.getDuckPosition());
        duckPosition = vision.stageSwitchingPipeline.getDuckPosition();
    }

    public void start() {
        telemetry.addData("Start:", "pressed");
    }

    int state = -1;
    boolean done = false;
    double extension1 = 0;
    double extension2 = 0;
    double retraction1 = 0;
    double retraction2 = 0;
    double placingPosition = 0;
    double drop = 0;
    double cmForward = 0;
    double cmReverse = 0;

    @Override
    public void loop() {
        telemetry.addData("state", state);
        if (duckPosition == "RIGHT") {
            extension1 = 65;
            extension2 = 0;
            retraction1 = 0;
            retraction2 = 65;
            placingPosition = 0.1;
            drop = 0.4;
            cmForward = 35;
            cmReverse = 5;
        } else if (duckPosition == "LEFT") {
            extension1 = 45;
            extension2 = 45;
            retraction1 = 45;
            retraction2 = 45;
            placingPosition = 0;
            drop = 0.25;
            cmForward = 42;
            cmReverse = 12;
        } else {
            extension1 = 45;
            extension2 = 0;
            retraction1 = 0;
            retraction2 = 45;
            placingPosition = 0;
            drop = 0.25;
            cmForward = 37;
            cmReverse = 7;
        }

        switch (state) {
            case -1:
                if (!done) {
                    done = robot.drive(-90, 40, 0.2);
                } else {
                    robot.stop();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 0:
                if (!done) {
                    done = robot.turnTo(20, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done) {
                    done = robot.linear.extendSlide(extension1, 0.5);
                } else {
                    robot.linear.stopExtend();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (getRuntime() < 1) {
                    robot.linear.servoTurn(placingPosition);
                } else {
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;

            case 3:
                if (!done) {
                    done = robot.linear.retractSlide(retraction1, 0.5);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done) {
                    done = robot.drive(180, cmForward, 0.2);
                } else {
                    robot.stop();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (getRuntime() < 1) {
                    robot.linear.basketMove(drop);
                } else {
                    done = false;
                    state++;
                }
                break;
            case 6:
                if (!done) {
                    done = robot.drive(0, cmReverse, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 7:
                if (!done) {
                    done = robot.linear.extendSlide(extension2, 0.5);
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
                    done = robot.linear.retractSlide(retraction2, 0.5);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 10:
                if (!done) {
                    done = robot.turnTo(82, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 11:
                if (!done) {
                    done = robot.drive(0, 90, 0.2);
                } else {
                    robot.stop();
                    resetStartTime();
                    done = false;
                    state++;
                }
                break;
            case 12:
                if (!done) {
                    done = robot.turnTo(170, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 13:
                if (!done) {
                    done = robot.drive(180, 14, 0.1);
                } else {
                    resetStartTime();
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 14:
                if (getRuntime() < 3) {
                    robot.carousel.autoSpin(0.4);
                } else {
                    robot.carousel.spinStop();
                    done = false;
                    state++;
                }
                break;
            case 15:
                if (!done) {
                    done = robot.drive(-10, 56, 0.1);
                } else {
                    resetStartTime();
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