package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision
{



// understand process frame method


    
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        stageSwitchingPipeline = new StageSwitchingPipeline();
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.setPipeline(stageSwitchingPipeline);
                webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
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
        Mat mat = new Mat();
        Telemetry telemetry;

        public enum Location {
            LEFT,
            RIGHT,
            NOT_FOUND
        }

        private Location location;
        /**
         * Rect creates areas in which the objects will be detected
         */

        static final Rect LEFT_ROI = new Rect(
                new Point(60, 35),
                new Point(120, 75));
        static final Rect RIGHT_ROI = new Rect(
                new Point(140, 35),
                new Point(200, 75));
        static double PERCENT_COLOR_THRESHOLD = 0.4;

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
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // can convert to YUV rather than RGB for better object detection
           // return input;

            /**
             * Scaler creates target range of color
             * X represents Hue
             * Y represents Saturation
             * Z represents a range of values
             */
            Scalar lowHSV =  new Scalar(23, 50, 70);
            Scalar highHSV = new Scalar(32, 255, 255);
            // variables for yellow

            Core.inRange(mat, lowHSV, highHSV, mat);

            Mat left = mat.submat(LEFT_ROI);
            Mat right = mat.submat(RIGHT_ROI);

            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 25;

            left.release();
            right.release();

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

            boolean  objLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean objRight = rightValue > PERCENT_COLOR_THRESHOLD;

            if(objLeft && objRight){
                location = Location.NOT_FOUND;
                telemetry.addData("Object Location", "not found");
            }
            else if(objLeft){
                location = Location.RIGHT;
                telemetry.addData("Object Location", "right");
            } else{
              location = Location.LEFT;
                telemetry.addData("Object Location", "left");
            }
            telemetry.update();

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            Scalar colorObj = new Scalar(255, 0, 0);
            Scalar colorObject = new Scalar(0, 255, 0);

            Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorObject:colorObj);
            Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorObject:colorObj);

            return mat;
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
