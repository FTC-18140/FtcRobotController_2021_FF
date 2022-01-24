package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="AutoLRed", group="Auto")
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
            /*case 0:
                if (getRuntime() < 10) {
                    robot.stop();
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;*/
            case 0:
                if (!done) {
                    done = robot.drive(180, 18, 0.75);
                } else {
                    resetStartTime();
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            /*case 1:
                if (getRuntime() < 1) {
                    robot.intake.intakeMove(1);
                } else {
                    robot.intake.intakeStop();
                    done = false;
                    state++;
                }
                break;
            case 2:
               if (!done) {
                   done = robot.linear.extend(35, 1);
                } else {
                   resetStartTime();
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (getRuntime() < 2) {
                    robot.linear.servoTurn();
                } else {
                    robot.linear.negativeServoTurn();
                    done = false;
                    state++;
                }
                break;
            */

            case 1:
                if (!done) {
                    done = robot.turn(-90, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (!done) {
                    done = robot.drive(0, 170, 0.75);
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

