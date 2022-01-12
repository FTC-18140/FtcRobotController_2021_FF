package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    CRServo leftServo = null;
    CRServo rightServo = null;
    HardwareMap hwMap = null;

    private Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry telem) {
        hwMap = ahwMap;
        telemetry = telem;

        leftServo = hwMap.crservo.get("ls");
        rightServo = hwMap.crservo.get("rs");
    }
    public void spin(double speed) {
        leftServo.setPower(speed);
        rightServo.setPower(speed);
    }
    public void spinStop() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}
