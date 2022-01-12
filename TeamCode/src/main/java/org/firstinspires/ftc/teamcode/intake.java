package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intake {
    DcMotor intakeMotor = null;
    HardwareMap hwMap = null;

    private Telemetry telemetry;
    public void init(HardwareMap ahwMap, Telemetry telem) {

        hwMap = ahwMap;
        telemetry = telem;

        intakeMotor = hwMap.dcMotor.get("intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public void intakeMove(double power) {
        intakeMotor.setPower(power);
    }
    public void intakeStop() {
        intakeMotor.setPower(0);
    }
}
