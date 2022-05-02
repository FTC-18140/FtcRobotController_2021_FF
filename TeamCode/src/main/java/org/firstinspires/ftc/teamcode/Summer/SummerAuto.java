package org.firstinspires.ftc.teamcode.Summer;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Thunderbot_2021;

@Autonomous(name="Auto", group="Summer")
public class SummerAuto extends OpMode
{
    SummerBot robot = new SummerBot();
    Path path = new Path();


    int state = 0;
    boolean done = false;


    public void init()
    {
        robot.init(hardwareMap,telemetry);
        path.addPoint(0,0);
        path.addPoint(10, 10);
        path.addPoint(20,10);
        path.addPoint( 20, 30);
        path.addPoint( 30, 30);
    }

    public void start()
    {

    }

    public void loop()
    {
        PVector pursuitVelocity = robot.pursuit.follow(path);
        robot.pursuitDrive(pursuitVelocity);
    }
}
