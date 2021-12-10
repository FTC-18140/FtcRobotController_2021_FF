package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "LinearSlideTest", group = "Auto")
public class LinearSlideTest extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
    telemetry.addData("Init =", " Done");
    robot.init(hardwareMap, telemetry);
    }

    public void start() {
    telemetry.addData("Starting", "...");
    }
    int state = 0;
    boolean done = false;

    @Override
        public void loop () {


                switch (state) {
                    case 0:
                        if (!done) {
                            telemetry.addData("starting =", "true");
                           done = robot.linear(10,-0.5);
                        } else {
                            robot.stop();
                            done = false;
                            state++;
                        }
                        break;
                    case 1:
                        if(!done) {
                            done = robot.linear(10, 0.5);
                        } else {
                            robot.stop();
                            done = false;
                            state++;
                        }
                    default:
                        break;
            }
        }
    }
