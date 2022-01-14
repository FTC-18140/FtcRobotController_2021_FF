package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto", group="Auto")
public class Auto extends OpMode {
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
                    try {
                        robot.carousel.autoSpin(5000,-1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done) {
                    robot.turn(-50, 0.25);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (!done) {
                    robot.drive(0, 20, 0.75);
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

