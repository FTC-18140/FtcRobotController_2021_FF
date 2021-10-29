package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="AutoRyan", group="Auto")
public class AutoRyan extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Init Done: ", "yes");
    }

    public void start() {

    }
        int state = 0;
        boolean done = false;
        public void loop () {
            switch (state) {
                case 0:
                    if (!done) {
                        done = robot.drive(30, 0.5);
                    } else {
                        robot.stop();
                        done = false;
                        state++;
                    }
                    break;

                case 1:
                    if (!done) {
                        done = robot.drive(30, -0.5);
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
