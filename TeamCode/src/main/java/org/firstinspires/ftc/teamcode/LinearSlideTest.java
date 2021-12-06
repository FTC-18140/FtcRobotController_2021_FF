package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "LinearSlideRyan", group = "Auto")
public class LinearSlideTest extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Init =", " Done");
    }

    public void start() {

    }
    int state = 0;
    boolean done = false;

    @Override
        public void loop () {


                switch (state) {
                    case 0:
                        if (!done) {
                            telemetry.addData("starting =", "true");
                            done = robot.theLift.move(10,-0.5);
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
