package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.tensorflow.lite.task.core.vision.ImageProcessingOptions;


public class Thunderbot_2021 {
    /**
     * Public OpMode members
     */
    // defines all varibles setting them to null
    BNO055IMU imu = null;
    DcMotor intakeMotor = null;
    LinearSlide linear = new LinearSlide();
    intake intake = new intake();

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    /**
     * local OpMode members
     */
    HardwareMap hwMap = null;
    private Telemetry telemetry;

    /**
     * Constructor
     */
    public Thunderbot_2021() {

    }

    /**
     * Initialize standard Hardware interfaces
     */
    public void init(HardwareMap ahwMap, Telemetry telem) {

        hwMap = ahwMap;
        telemetry = telem;

        linear.init(hwMap, telemetry);

        intake.init(hwMap, telemetry);
    }

}



