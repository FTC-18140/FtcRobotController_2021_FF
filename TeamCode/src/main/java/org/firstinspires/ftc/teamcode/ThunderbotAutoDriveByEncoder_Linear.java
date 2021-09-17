/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;


// slow down some parts of the code and try to lower some cool downs
// add a sensor  to increase accuracy (light sensor, gyro for strafe, voltage, etc.)

@Autonomous(name="Thunderbot: Auto Drive By Encoder", group="Thunderbot")
//@Disabled
public class ThunderbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Thunderbot robot = new Thunderbot();   // Use a Thunderbots hardware
    int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive system variables.
        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);
        robot.init(hardwareMap, telemetry);

        if (robot.tfod != null) {
            robot.tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            robot.tfod.setZoom(2.0, 16.0/9.0);
        }

        //robot.checkRings();
        while (!opModeIsActive()) { // fixed version of checkRings method
                if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                        robot.nOfSetRings = 0;
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                robot.nOfSetRings = 1;
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                robot.nOfSetRings = 4;
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }
                    }
                }
                telemetry.update();
            }
        }

        // Wait for the game to start (driver presses PLAY)ds
        waitForStart();

        // Note: use sleep when you want the robot to stop for a selected time
        while (opModeIsActive()){
            if (robot.tfod != null) {
                robot.tfod.shutdown();
            }

            if (robot.nOfSetRings == 0){
                // square A
                robot.powerShot();
                robot.targetZoneA();
            } else if (robot.nOfSetRings == 1) {
                // square B
                robot.powerShot();
                robot.targetZoneB();
            } else if (robot.nOfSetRings == 4){
                // square C
                robot.powerShot();
                robot.targetZoneC();
            } else {
                // normal master code or square A
                robot.powerShot();
            }

            break;
        }

            telemetry.addData("Path", "Complete");
            telemetry.update();

    }
}