
package org.firstinspires.ftc.teamcode.Summer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This class provides basic Telop driving for a Summerbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common SummerBot hardware class to define the devices on the robot.
 * All device access is managed through the SummerBot class.
 */

@TeleOp(name="SummerTeleop", group="Summer")
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
        robot.init(hardwareMap, telemetry);

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
        resetStartTime();
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        // Tell the robot to update its position, velocity
        robot.update();

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;

        robot.joystickDrive(forward, right, clockwise);

//        telemetry.addData("forward: ",  "%.2f", forward);
//        telemetry.addData("right: ", "%.2f", right);
//        telemetry.addData("clockwise: ", "%.2f", clockwise);

        telemetry.addData("Loop Time: ", "%.4f", getRuntime());
        resetStartTime();

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
