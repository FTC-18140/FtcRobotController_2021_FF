package org.firstinspires.ftc.teamcode;


import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Auto", group="Auto")
//@Disabled
public class Auto extends OpMode {
    Thunderbot_2021 robot = new Thunderbot_2021();
    int barcode = 1;
    OpenCvCamera webcam;

    public void init() {
        robot.init(hardwareMap,telemetry);

    }

    @Override
    public void init_loop()
    {
        super.init_loop();
        barcode = robot.getBarcode();

        int cameraMonitorView = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera
                (OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorView);

        Vision detector = new Vision(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, UPRIGHT));

    }

    public void start(){

    }


    int state = 0;
    boolean done = false;
    public void loop() {
        switch (state) {
            case 0: // Drive diagonal 45 degrees
            if (!done) {
                done = robot.drive(45, 80, 0.2);
            } else {
                robot.stop();
                done = false;
                state++;
            }
            break;

            case 1: // Forward 30cm
                if (!done) {
                    done = robot.drive(0, 30, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 2: // Backward 30cm
                if (!done) {
                    done = robot.drive(0, 30, -0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 3: // Turn 90 Degrees
                if (!done) {
                    done = robot.turn(-90, 0.1);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            default:
                break;
        }
    }
}
