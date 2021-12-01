package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto", group="Auto")
//@Disabled
public class Auto extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();
    int barcode = 1;

    public void init() {
        robot.init(hardwareMap,telemetry);

    }

    @Override
    public void init_loop()
    {
        super.init_loop();
        barcode = robot.getBarcode();

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

            case 3: // Turn 90 Degrees
                if (!done) {
                    done = robot.turn(-90, 0.1);
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
