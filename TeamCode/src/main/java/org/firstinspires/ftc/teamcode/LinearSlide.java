package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    DcMotor linearSlideR = null;
    DcMotor linearSlideL = null;
    HardwareMap hwMap = null;

    private Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry telem) {
        hwMap = ahwMap;
        telemetry = telem;

        linearSlideR = hwMap.dcMotor.get("linearL");
        linearSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideL = hwMap.dcMotor.get("linearR");
        linearSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void extend(double power) {
        linearSlideL.setPower(power);
        LinearSlideR.setPower(power);
    }
}
