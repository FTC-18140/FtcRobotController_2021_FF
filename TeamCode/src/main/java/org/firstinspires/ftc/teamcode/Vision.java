package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

public class Vision
{

    /**
     * The webcam object which will grab the frames for us
     */
    OpenCvCamera webCam;
    /**
     * Local reference to the telemetry object for displaying debug info
     */
    Telemetry telemetry;
    /**
     * Implements the Barcode processing algorithm.
     */
    //CameraVision.StageSwitchingPipeline stageSwitchingPipeline;
    /**
     * The ID for the camera we are using (there are 2 on the robot)
     */
    public String camDeviceName = "webcam";

    /**
     * Initializes the Vision class
     * @param ahwMap -
     * @param telem -
     */
    public void init (HardwareMap ahwMap, Telemetry telem)
    {
        // Save reference to telemetry object
        telemetry = telem;

        // initialize the camera stuff here.
        // initialize the pipeline here.
    }

    /**
     * Returns the barcode read by the camera.
     * @return int representing the barcode level: 1, 2, or 3
     */
    public int getBarcode()
    {
        return 1;
    }

    // Need a Pipeline Class.


}
