package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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
    Vision.StageSwitchingPipeline stageSwitchingPipeline;
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

        /*
         * Look at the examples from EasyOpenCV and copy all the relevant initialization code here.
         */

    }

    /**
     * Returns the barcode read by the camera.
     * @return int representing the barcode level: 1, 2, or 3
     */
    public int getBarcode()
    {
        // return the results from the pipeline.
        return stageSwitchingPipeline.pipelineBarcode;
    }

    /**
     * Implements a Barcode detection algorithm as an OpenCV pipeline.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        int pipelineBarcode = 1;
        /**
         * Called when a frame from the webcam is returned.  Processing and looking for the barcode
         * happens in this method.
         *
         * @param input the Mat object which is the frame of video
         * @return a Mat object which has been updated with the results of the calculation for the barcode.
         */
        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Lots of stuff here.
             */
            return input;
        }

        /**
         * Cycle through which stage of the pipeline is displayed on the screen when the screen is tapped.
         */
        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */
        }

    }


}
