package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    DcMotor carousel = null;
    HardwareMap hwMap = null;

    private Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry telem) {
        hwMap = ahwMap;
        telemetry = telem;

        carousel = hwMap.dcMotor.get("carousel");
    }
    public void spin(double speed) {
        carousel.setPower(speed);
    }
    public void autoSpin(double power){
        carousel.setPower(power);
    }
    public void spinStop() {
        carousel.setPower(0);
    }
}
