package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CappingTurret {
    Servo xServo = null;
    Servo yServo = null;
    CRServo tapeServo = null;
    HardwareMap hwMap = null;

    private Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry telem) {
        hwMap = ahwMap;
        telemetry = telem;

        xServo = hwMap.servo.get("place holder");
        yServo = hwMap.servo.get("place holder");
        tapeServo = hwMap.crservo.get("place holder");

        /*xServo.setPosition(0);
        yServo.setPosition(0);*/
    }
}