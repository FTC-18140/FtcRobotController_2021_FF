package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class Vision
{
    HardwareMap hwMap;
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
    //public String camDeviceName = "webcam";

    /**
     * Initializes the Vision class
     * @param ahwMap -
     * @param telem -
     */
    public void init (HardwareMap ahwMap, Telemetry telem)
    {
        // Save reference to telemetry object
        hwMap = ahwMap;
        telemetry = telem;
/**
 * NOTE: Many comments have been omitted from this sample for the
 * sake of conciseness. If you're just starting out with EasyOpenCv,
 * you should take a look at {@link InternalCamera1Example} or its
 * webcam counterpart, {@link WebcamExample} first.
 */

        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(ahwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        stageSwitchingPipeline = new StageSwitchingPipeline();

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.setPipeline(stageSwitchingPipeline);
                webCam.startStreaming(432 , 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        /*int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class,"Webcam 1") ,cameraMonitorViewId);

        stageSwitchingPipeline = new StageSwitchingPipeline();

        webCam.setPipeline(stageSwitchingPipeline);
        //webCam.setMilisecondsPermisionTimout(2500);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                *//*
                  This will be called if the camera could not be opened
                 *//*
            }
        });*/
        /*
         * Look at the examples from EasyOpenCV and copy all the relevant initialization code here.
         */

    }

    /**
     * Implements a Barcode detection algorithm as an OpenCV pipeline.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        String duckPosition;
        static double leftSum = 0;
        static double rightSum = 0;
        static double centerSum = 0;

        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }

        private PipelineStageSwitchingExample.StageSwitchingPipeline.Stage stageToRenderToViewport = PipelineStageSwitchingExample.StageSwitchingPipeline.Stage.YCbCr_CHAN2;
        private PipelineStageSwitchingExample.StageSwitchingPipeline.Stage[] stages = PipelineStageSwitchingExample.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        /**
         * Called when a frame from the webcam is returned.  Processing and looking for the barcode
         * happens in this method.
         *
         * @param input the Mat object which is the frame of video
         * @return a Mat object which has been updated with the results of the calculation for the barcode.
         */
        @Override
        public Mat processFrame(Mat input) {

            /**
             * Scaler creates target range of color
             * X represents Hue
             * Y represents Saturation
             * Z represents a range of values
             */
            Scalar green = new Scalar(0, 250, 0);
            Scalar blue = new Scalar(0, 0, 255);
            Scalar red = new Scalar(255, 0, 0);
            contoursList.clear();

            int leftL = 20;
            int leftR = 140;
            int rightL = 280;
            int rightR = 400;

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 90, 255, Imgproc.THRESH_BINARY_INV);


            leftSum = Core.sumElems(thresholdMat.submat(96, 144, leftL, leftR)).val[0];
            rightSum = Core.sumElems(thresholdMat.submat(96, 144, rightL, rightR)).val[0];
            centerSum = Core.sumElems(thresholdMat.submat(96, 144, leftR, rightL)).val[0];

            if(leftSum > rightSum && leftSum > centerSum){
                Imgproc.rectangle(
                        input,
                        new Point(
                                leftL,
                                85),
                        new Point(
                                leftR,
                                170),
                        blue, 4);
                duckPosition = "LEFT";
            } else if (centerSum > leftSum && centerSum > rightSum){
                Imgproc.rectangle(
                        input,
                        new Point(
                                leftR +10,
                                85),
                        new Point(
                                rightL - 10,
                                170),
                        red, 4);
                duckPosition = "CENTER";
            } else {
                Imgproc.rectangle(
                        input,
                        new Point(
                                rightL,
                                85),
                        new Point(
                                rightR,
                                170),
                        green, 4);
                duckPosition = "RIGHT";
            }
            /*switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }*/

            Imgproc.rectangle(
                    thresholdMat,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);
            //return yCbCrChan2Mat;
            return input;
            //return thresholdMat;

        }

        /**
         * Returns the barcode read by the camera.
         * @return int representing the barcode level: 1, 2, or 3
         */
        public String getDuckPosition()
        {
            return duckPosition;
        }
    }
}


