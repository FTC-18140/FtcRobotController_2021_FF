package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto", group="Auto")
//@Disabled
public class Auto extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();

    public void init() {
        robot.init(hardwareMap,telemetry);

    }

    public void start(){

    }


    int state = 0;
    boolean done = false;
    public void loop() {
        switch (state) {
            case 0: // Drive diagonal 45 degrees
            if (!done) {
                done = robot.drive(45, 80, 0.2);
            } else {
                robot.stop();
                done = false;
                state++;
            }
            break;

            case 1: // Forward 30cm
                if (!done) {
                    done = robot.drive(0, 30, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 2: // Backward 30cm
                if (!done) {
                    done = robot.drive(0, 30, -0.2);
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
