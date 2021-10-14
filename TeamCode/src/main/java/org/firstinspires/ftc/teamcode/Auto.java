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

    boolean done = false;
    public void loop() {
        if(!done){
            done = robot.drive(100, 0.5);
        }
    }
}
