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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

    BNO055IMU imu = null;

    HardwareMap hwMap =  null;
    Telemetry telemetry = null;

    boolean moving = false;
    double gyStartAngle = 0;
    double initialPosition = 0;

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    /**
     * Robot - Encoder information used in odometry
     */
    /** An x-axis velocity measurement. */
    private double xEncInPerSec;
    /** A y-axis velocity measurement. */
    private double yEncInPerSec;
    /** The radius of the encoder wheels. */
    private final double encWheelRadius = 1.96/2.0; //in inches ... encoder is a 50mm diameter wheel.
//    private final double encTickPerRotation = 2400;
    /** The number of encoder ticks per rotation of the encoder wheel. */
    private final double encTickPerRotation = 3200;
//    public static double encDistanceConstant = 195.5/192; //calibrated over 16' & 12' on foam tiles -- 9/13/19
    /** A constant used in calculating robot position change. */
    public static double encDistanceConstant = 1;
    /** The number of inches moved per rotation of the encoder wheel. */
    private final double encInchesPerRotation = 2.0 * encWheelRadius * Math.PI * encDistanceConstant; // this is the encoder wheel distancd
    /** The gearing on the drive train. */
    private final double gearRatio = 1.733333333;
    /** The number of encoder ticks per inch moved. */
    private final double encTicksPerInch = encTickPerRotation / (encInchesPerRotation);

    /** Used to calculate the change in x position. */
    private int prevXEncoder = 0;
    /** Used to calculate the change in y position. */
    private int prevYEncoder = 0;
    /** Stores the change in x position. */
    private int xEncoderChange = 0;
    /** Stores the change in y position. */
    private int yEncoderChange = 0;

    /**
     * Robot - Odometry Items
     */
    /** Declares the xEncoder as an expanded rev hub motor. */
    private DcMotorEx xEncoder = null;
    /** Declares the yEncoder as an expanded rev hub motor. */
    private DcMotorEx yEncoder = null;

    // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
    List<LynxModule> allHubs = null;

    /**
     * PathFollower object - used to give the robot the ability to follow a sequence of
     * points defined beforehand.
     */
    PathFollower pursuit = new PathFollower( new PVector(0,0), 10, 10);

    /**
     *  Constructor
     */
    public SummerBot()
    {

    }

    /**
     *  Initialize the hardware using the reference to the HardwareMap.
     */
    public void init(HardwareMap ahwMap, Telemetry telem)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;

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

        try
        {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU.
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        // Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        // Set all Expansion hubs to use the MANUAL Bulk Caching mode
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     *Queries the REV Hub using bulk data transfer.  Updates
     * the heading and position values.
     */
    public void update()
    {
        // Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }

        /*
         * Update the velocity as read by the encoders
         */
        double xEncTicksPerSecond = xEncoder.getVelocity() / gearRatio;
        double yEncTicksPerSecond = yEncoder.getVelocity() / gearRatio;

        xEncInPerSec = xEncTicksPerSecond / encTicksPerInch;
        yEncInPerSec = yEncTicksPerSecond / encTicksPerInch;

        /*
         * Update the position change since the last time as read by the encoders
         */
        xEncoderChange = xEncoder.getCurrentPosition() - prevXEncoder;
        yEncoderChange = yEncoder.getCurrentPosition() - prevYEncoder;

        updateVelocity(this.getVelocity());
        updatePosition(this.getLocationChange());

        /* store the current value to use as the previous value the next time around */
        prevXEncoder = xEncoder.getCurrentPosition();
        prevYEncoder = yEncoder.getCurrentPosition();
    }

    /**
     * This code go's through the math behind the mecanum wheel drive
     * @param foward - Any forward motion including backwards
     * @param right - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise)
    {
        double frontLeft = foward + clockwise + right;
        double frontRight = foward - clockwise - right;
        double backLeft = foward + clockwise - right;
        double backRight = foward - clockwise + right;

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
     * @param neededVelocity PVector that indicates the linear velocity needed by the robot to continue pursuit
     * @param spin double indicating the angular velocity needed by the robot to continue pursuit
     */
    public void updateMotors(PVector neededVelocity, double spin)
    {
        // Fix this next line.  The robot should tank drive for TBD.
        // neededVelocity.rotate((float)Math.toRadians( getHeadingPursuit() ));
        double x = neededVelocity.x / 40.0; //bot.maxSpeed; //max speed is 31.4 in/sec
        double y = neededVelocity.y / 40.0; // bot.maxSpeed;

//        telemetry.addData("x encoder: ", xEncoder.getCurrentPosition());
//        telemetry.addData("y encoder: ", yEncoder.getCurrentPosition());
        telemetry.addData("SbfJoystick x, y: ", "%.3f, %.3f", x, y );

        double turn = spin / 343;

        joystickDrive(x, y, turn );
    }

    /**
     * Get the heading in whatever means you are implementing.  Right now, use the imu.
     * @return heading in degrees
     */
    public double updateHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    /** Update the current location of the robot.
     * @param currentPosition The position being added to location.
     * */
    public void updatePosition(PVector currentPosition)
    {
        pursuit.location = PVector.add(pursuit.location, currentPosition);
    }

    /**
     * Calculate the amount of X distance the robot has travelled since the last time this was called
     * @return float indicating the number of inches traveled
     */
    private float getXChange()
    {
        double inchesX = (((xEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.cos(Math.toRadians(updateHeading())) +
                (((yEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.sin(Math.toRadians(updateHeading()));
        return (float)inchesX;
    }

    /**
     * Calculate the amount of Y distance the robot has travelled since the last time this was called
     * @return float indicating the number of inches traveled
     */
    private float getYChange()
    {
        double inchesY = ((-(xEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.sin(Math.toRadians( updateHeading() )) +
                (((yEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.cos(Math.toRadians( updateHeading() ));
        return (float)inchesY;
    }
    /**
     * Update the current velocity.
     * @param currentVelocity  The velocity being set.
     */
    public void updateVelocity(PVector currentVelocity)
    {
        pursuit.velocity.set(currentVelocity.x, currentVelocity.y);
    }

    /**
     * Robot - Get the robot's current linear velocity
     * @return PVector which captures the current x,y velocity of the robot.
     */
    public PVector getVelocity()
    {
        return new PVector(getXLinearVelocity(), getYLinearVelocity());
    }

    /**
     * Access the current linear velocity in the X direction.
     * @return float representing the X linear velocity in in/sec
     */
    private float getXLinearVelocity()
    {
        double linearX = xEncInPerSec * Math.cos(Math.toRadians( updateHeading() )) +
                yEncInPerSec * Math.sin(Math.toRadians( updateHeading() ));
        return (float)linearX;
    }

    /**
     * Robot - Access the current linear velocity in the Y direction.
     * @return float representing the Y linear velocity in in/sec
     */
    private float getYLinearVelocity()
    {
        double linearY = -xEncInPerSec * Math.sin(Math.toRadians( updateHeading() )) +
                (yEncInPerSec) * Math.cos(Math.toRadians( updateHeading() ));
        return (float)linearY;
    }

    /**
     * Robot - calculate the amount of distance the robot has travelled since the last time this was called
     * @return PVector indicating the number of inches traveled in x,y
     */
    public PVector getLocationChange()
    {
        return new PVector(getXChange(), getYChange());
    }

    public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }
    public void resetLeftTicks() {
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }
    public int getLeftTicks() {
        return leftEncoderMotor.getCurrentPosition() - leftEncoderPos;
    }
    public void resetRightTicks() {
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }
    public int getRightTicks() {
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }
    public void resetCenterTicks() {
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }
    public int getCenterTicks() {
        return centerEncoderMotor.getCurrentPosition() - centerEncoderPos;
    }
    public void drive(double fl, double bl, double fr, double br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }
    public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        resetTicks();
    }

}

