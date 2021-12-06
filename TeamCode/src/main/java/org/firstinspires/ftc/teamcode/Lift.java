package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

public class Lift
{
    DcMotor LinearSlide = null;
    boolean moving = false;
    int initial = 0;

    HardwareMap hwMap = null;
    private Telemetry telemetry;


    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;

        LinearSlide = hwMap.dcMotor.get("LinearSlide");
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean move(double distance2, double power2) {

        if (!moving) {
            initial = LinearSlide.getCurrentPosition();
            moving = true;
        }

        double position2 = abs(LinearSlide.getCurrentPosition() - initial);
        double positionInCM2 = position2 / Thunderbot_2021.COUNTS_PER_CM;

        if (positionInCM2 >= distance2) {
            stop();
            moving = false;
            return true;
        }
        else {
            LinearSlide.setPower( power2 );
            return false;
        }
    }

    // Stop all motors
    public void stop() {
        LinearSlide.setPower(0);
    }

}
