package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    //member objects
    CRServo carouselRight = null;
    CRServo carouselLeft = null;

    HardwareMap hwMap = null;
    private Telemetry telemetry;

    //constructer
    public Carousel() {
    }

    //class methods
    //servo init
    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;

        carouselRight = hwMap.crservo.get("cr");
        carouselLeft = hwMap.crservo.get("cl");
    }

    //method to make servo turn
    public void spin(double speed) {
        carouselLeft.setPower(speed);
        carouselRight.setPower(speed);

    }
    //stops servo
    public void stop() {
        carouselRight.setPower(0.0);
        carouselLeft.setPower(0.0);
    }

}
