/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Summer;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import static java.lang.Math.abs;

/**
 * This is NOT an OpMode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a SummerBot.
 *
 */
public class SummerBot
{
    /* Motors used in the robot chassis */
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    HardwareMap hwMap =  null;
    Telemetry telemetry = null;

    boolean moving = false;
    double gyStartAngle = 0;
    double initialPosition = 0;

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28.0; // rev robotics hd hex motors planetary-411600
    static final double DRIVE_GEAR_REDUCTION = 18.88;  // REV 3.61 * 5.23 planetary stages
    static final double WHEEL_DIAMETER_CM = 96.0/10.0;  // GoBilda 96mm mecanum wheels-3213-3606-0002
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.141593);

    //
    // Robot - Encoder information used in odometry
    //
    /** Encoder velocity measurement. */
    private double leftEncInPerSec;
    private double rightEncInPerSec;
    private double backEncInPerSecond;

    /** The radius of the encoder wheels. */
    private final double encWheelRadius = 1.96/2.0; //in inches ... encoder is a 50mm diameter wheel.
    /** The number of encoder ticks per rotation of the encoder wheel. */
    private final double encTickPerRotation = 3200;
    /** A constant used in calculating robot position change. */
    public static double encDistanceConstant = 1;
    /** The number of inches moved per rotation of the encoder wheel. */
    private final double encInchesPerRotation = 2.0 * encWheelRadius * Math.PI * encDistanceConstant; // this is the encoder wheel distance
    /** The gearing on the drive train. */
    private final double gearRatio = 1.733333333;

    /** Declares the encoders as an expanded rev hub motor. */
    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx backEncoder = null;

    /** The number of encoder ticks per inch moved. */
    private final double encTicksPerInch = encTickPerRotation / (encInchesPerRotation);

    /** Used to calculate the change in position. */
    private int prevLeftEncoder = 0;
    private int prevRightEncoder = 0;
    private int prevBackEncoder = 0;

    /** Stores the change in  position. */
    private int leftEncoderChange = 0;
    private int rightEncoderChange = 0;
    private int backEncoderChange = 0;

    /** locations for encoders */
    private final double h = 6;
    private final double D = 14;

    /**
     * Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
     */
    List<LynxModule> allHubs = null;

    /**
     * PathFollower object - used to give the robot the ability to follow a sequence of
     * points defined beforehand.
     */
    PathFollower pursuit = null;

    /**
     *  Constructor
     */
    public SummerBot()
    {

    }

    /**
     *  Initialize the hardware using the reference to the HardwareMap.
     */
    public void init(HardwareMap theHwMap, Telemetry theTelemetry)
    {
        // Save reference to Hardware map
        hwMap = theHwMap;
        telemetry = theTelemetry;

        pursuit = new PathFollower( new PVector(0,0), 10, 10, telemetry );

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "lf");
        rightFront = hwMap.get(DcMotorEx.class, "rf");
        leftRear = hwMap.get(DcMotorEx.class, "lr");
        rightRear = hwMap.get(DcMotorEx.class, "rr");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        stop();

        // Use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the motors
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set up the encoders
        leftEncoder = hwMap.get(DcMotorEx.class, "leftEncoder");
        leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setDirection((DcMotorEx.Direction.REVERSE));

        rightEncoder = hwMap.get(DcMotorEx.class, "rightEncoder");
        rightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setDirection((DcMotorEx.Direction.FORWARD));

        backEncoder = hwMap.get(DcMotorEx.class, "backEncoder");
        backEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setDirection((DcMotorEx.Direction.REVERSE));

        // Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        // Set all Expansion hubs to use the MANUAL Bulk Caching mode
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Queries the REV Hub using bulk data transfer.  Updates
     * the heading and position values.
     */
    public void update()
    {
        // Important Step: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }
        updateRobotVelocity();
        updateRobotPosition();
    }

    /**
     * This code go's through the math behind the mecanum wheel drive
     * @param forward - Any forward motion including backwards
     * @param right - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double forward, double right, double clockwise)
    {
        double frontLeft = forward + clockwise + right;
        double frontRight = forward - clockwise - right;
        double backLeft = forward + clockwise - right;
        double backRight = forward - clockwise + right;

        double max = abs(frontLeft);
        if (abs(frontRight) > max)
        {
            max = abs(frontRight);
        }
        if (abs(backLeft) > max)
        {
            max = abs(backLeft);
        }
        if (abs(backRight) > max)
        {
            max = abs(backRight);
        }
        if (max > 1)
        {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        leftFront.setPower(frontLeft);
        rightFront.setPower(frontRight);
        leftRear.setPower(backLeft);
        rightRear.setPower(backRight);
    }

    /**
     * gyroDrive - drive at a particular heading
     * @return boolean value indicating the drive is complete
     */

    public boolean gyroDrive (double direction, double distance, double power)
    {

        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

        // Set desired angle and initial position
        if( !moving )
        {
            gyStartAngle = direction;

            // If my intended direction to drive is negative, and it's close enough to -180 to be worried,
            // add 360 degrees to it. This will prevent the angle from rolling over to +/-180.
            // For example, if my desired direction is -165, I would add 360 to that, and my new
            // desired direction would be 195.
            if (gyStartAngle < 0.0 && Math.abs(gyStartAngle) > 130.0 )
            {
                gyStartAngle = gyStartAngle + 360;
            }

            if(direction == 45)
            {
                // the rightFront wheel doesn't move at a desired direction of 45 degrees
                initialPosition = leftFront.getCurrentPosition();
            }
            else
            {
                initialPosition = rightFront.getCurrentPosition();
            }
            moving = true;
        }

        double position;
        if(direction == 45)
        {
            position = abs(leftFront.getCurrentPosition() - initialPosition);
        }
        else
        {
            position = abs(rightFront.getCurrentPosition() - initialPosition);
        }

        double positionInCM = position/COUNTS_PER_CM;
        telemetry.addData("position", position);

        if (Math.abs(gyStartAngle) > 130 && currentAngle < 0.0)
        {
            // Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

        // calculates required speed to adjust to gyStartAngle
        double adjust = (gyStartAngle - currentAngle ) / 100;
        // Setting range of adjustments
        adjust = Range.clip(adjust, -1, 1);

        // Stops when at the specified distance
        if(positionInCM >= distance)
        {
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            joystickDrive(power, 0, adjust);
            return false;
        }
    }

    /**
     * This method sets all motor power to 0, causing the robot to stop.
     */
    public void stop()
    {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Robot - command the Chassis to move according to the desired linear velocity and spin calculated
     * by the pursuit code.
     */
    public void pursuitDrive(PVector pursuitVelocity)
    {
        double forward = pursuitVelocity.mag() / 40.0; // the maximum speed of the robot
        forward = Range.clip(forward, -1, 1);

        // calculates required speed to adjust to angle for the pursuitVelocity
        double turn = (pursuit.velocity.heading() - pursuitVelocity.heading()) / 100;
        turn = Range.clip(turn, -1, 1);

        telemetry.addData("Joystick forward, right, turn: ", "%.3f, %.3f", forward, 0.0, turn );

        joystickDrive(forward, 0, turn );
    }

    /**
     * Get the heading in whatever means you are implementing.  Right now, use the imu.
     * @return heading in degrees
     */
    private double updateHeading()
    {
        double deltaLeft = leftEncoderChange / encInchesPerRotation * encInchesPerRotation;
        double deltaRight = rightEncoderChange / encInchesPerRotation * encInchesPerRotation;

        double heading = (deltaRight - deltaLeft ) / D;
        telemetry.addData("Robot Heading: ", heading);
        return heading;
    }

    /**
     * Update the current location of the robot
     */
    private void updateRobotPosition()
    {

        //
        // Update the position change since the last time as read by the encoders
        //
        leftEncoderChange = leftEncoder.getCurrentPosition() - prevLeftEncoder;
        rightEncoderChange = rightEncoder.getCurrentPosition() - prevRightEncoder;
        backEncoderChange = backEncoder.getCurrentPosition() - prevBackEncoder;

        double deltaBack = backEncoderChange / encTickPerRotation * encInchesPerRotation;
        double deltaLeft = leftEncoderChange / encInchesPerRotation * encInchesPerRotation;
        double deltaRight = rightEncoderChange / encInchesPerRotation * encInchesPerRotation;

        double globalXChange =  deltaBack + (deltaRight - deltaLeft)*h/D;
        double globalYChange = ( deltaLeft + deltaRight ) / 2.0;

        PVector positionChange = new PVector((float)globalXChange, (float)globalYChange);
        pursuit.setPosition( positionChange );

        telemetry.addData("Robot Position: ", pursuit.location);

        //
        // Store the current value to use as the previous value the next time around
        //
        prevLeftEncoder = leftEncoder.getCurrentPosition();
        prevRightEncoder = rightEncoder.getCurrentPosition();
        prevBackEncoder = backEncoder.getCurrentPosition();
    }

    /**
     * Update the current velocity.
     */
    private void updateRobotVelocity()
    {
        //
        //  Update the velocity as read by the encoders
        //
        double leftEncTicksPerSecond = leftEncoder.getVelocity() / gearRatio;
        double rightEncTicksPerSecond = rightEncoder.getVelocity() / gearRatio;
        double backEncTicksPerSecond = backEncoder.getVelocity() / gearRatio;

        leftEncInPerSec = leftEncTicksPerSecond / encTicksPerInch;
        rightEncInPerSec = rightEncTicksPerSecond / encTicksPerInch;
        backEncInPerSecond = backEncTicksPerSecond / encTicksPerInch;

        double globalXVelocity = backEncInPerSecond + (rightEncInPerSec - leftEncInPerSec)/D*h;
        double globalYVelocity = ( leftEncInPerSec + rightEncInPerSec ) / 2;

        PVector currentVelocity = new PVector((float)globalXVelocity, (float)globalYVelocity);
        pursuit.setVelocity( currentVelocity);

        telemetry.addData("Robot Velocity: ", pursuit.velocity);
    }


}

