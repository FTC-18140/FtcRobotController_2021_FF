package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="HoldingFreight", group="Auto")
public class HoldingFreight extends OpMode {
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
                    done = robot.linear.extendSlide(74, 0.5);
                    robot.linear.servoTurn(0.45);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;

            /*case 0:
                if (!done) {
                    robot.linear.servoTurn(0.45);
                } else {
                    robot.linear.stopExtend();
                    done = false;
                    state++;
                }
                break;*/

            default:
                break;
        }
    }
}
