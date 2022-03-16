
package org.firstinspires.ftc.teamcode.Summer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Summer.SummerBot;


/**
 * This class provides basic Telop driving for a Summerbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common SummerBot hardware class to define the devices on the robot.
 * All device access is managed through the SummerBot class.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Summer.SummerBot;

@TeleOp(name="Teleop", group="Summer")
public class SummerTeleop extends OpMode
{

    /* Declare OpMode members. */
    SummerBot robot = new SummerBot(); // use the class created to define a Pushbot's hardware

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Complete");    //
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        // Do nothing
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        double forward;
        double right;
        double clockwise;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        clockwise = gamepad1.right_stick_x;

        robot.drive(forward, clockwise, right);

        telemetry.addData("forward: ",  "%.2f", forward);
        telemetry.addData("right: ", "%.2f", right);
        telemetry.addData("clockwise: ", "%.2f", clockwise);
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        robot.stop();
    }
}
