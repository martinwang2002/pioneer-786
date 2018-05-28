/* Copyright (c) 2017 QDHS Pioneer.
 * Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.Pioneer2018;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

// VER 1

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "sensor_color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "BLUE", group = "Auto")
//@Disabled
public class AutoB extends LinearOpMode {

    ColorSensor sensorRGB;

    private Servo servoRight = null;
    private Servo servoLeft = null;
    private Servo servoPole = null;
    private Servo servoPole_secdonary = null;
    private Servo servoOffload = null;

    private DcMotor leftFDrive = null;
    private DcMotor rightFDrive = null;
    private DcMotor rightBDrive = null;
    private DcMotor leftBDrive = null;

    private DcMotor intakeL = null;
    private DcMotor intakeR = null;

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;

    private int state = 0;
    // 00 = waiting
    // 10 = pole down
    // 20 = ball - S1
    // 21 = ball - S2
    // 30 = off platform
    // 40 = spit block
    // 99 = end

    private void run(double drivex, double drivey, double turn, double velocity, double ang_velocity) {
        double[] v0 = MechanismUtil.calcv(turn * ang_velocity, drivex, drivey);

        double rightFPower = Range.clip(v0[0] * velocity, -1.0, 1.0);
        double leftFPower = Range.clip(v0[1] * velocity, -1.0, 1.0);
        double leftBPower = Range.clip(v0[2] * velocity, -1.0, 1.0);
        double rightBPower = Range.clip(v0[3] * velocity, -1.0, 1.0);

        // Send calculated power to wheels
        leftFDrive.setPower(Math.abs(leftFPower) * leftFPower);
        rightFDrive.setPower(Math.abs(rightFPower) * rightFPower);
        leftBDrive.setPower(Math.abs(leftBPower) * leftBPower);
        rightBDrive.setPower(Math.abs(rightBPower) * rightBPower);
    }

    @Override
    public void runOpMode() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AVPqpOv/////AAADmQKVp/KL0E0lkd0l63ZUA7IA46W6F+fpvg6O3Yl/2DXqIDmWYhMiqV2kq3YkmcgIJPcAYIxaDs3N3BCyrOFk2K8dDHAJ3ALP56hlTI7jf9DH2UaB6L8tmyLLMNQDYQVNIaT1aiipG5mLG82v4I2IBErXwlBBNmiaM65KkPZ2nNP6ASFzsSW8yOPP3A1y7umbVOXrijrbE7FIY8JshnyM0EggMkI8H0WzQ617nTthYC3zhqCkquED4CEVGsxrjFwaqWr567fW2h391+DRQLMw2ZIudLTPq1djEl79FAT8mRkmGqbgJWcT0NKwgqQsxAjMC0QDlDMbND+7PPWHPjC/2Hyj4/u0iAnv02fcNs3bJvPy";
        //BuildConfig.VUFORIA_KEY;

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        leftFDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBDrive = hardwareMap.get(DcMotor.class, "rb");

        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBDrive.setDirection(DcMotor.Direction.REVERSE);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        servoPole = hardwareMap.get(Servo.class, "servoPole");
        servoPole_secdonary = hardwareMap.get(Servo.class, "servoPole2");

        servoOffload = hardwareMap.get(Servo.class, "platform_servo");

        // wait for the start button to be pressed.
        waitForStart();

        servoRight.setPosition(0.0);
        servoLeft.setPosition(0.55);

        servoPole.setPosition(1.0);
        servoPole_secdonary.setPosition(0.5);

        servoOffload.setPosition(0.0555);

        boolean ball_is_blue = false;
        double start_time = getRuntime();
        int vumark = -1;
        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            telemetry.addData("Current State:", state);

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (vuMark.toString().compareTo("LEFT") == 0) {
                    vumark = 0;
                } else if (vuMark.toString().compareTo("CENTER") == 0) {
                    vumark = 1;
                } else if (vuMark.toString().compareTo("RIGHT") == 0) {
                    vumark = 2;
                }

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", formatMatrix(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            if (getRuntime() > start_time + 1.0 && state < 10) {
                state = 10;
            }

            if (state == 10) {
                // convert the RGB values to HSV values.
                Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("Clear", sensorRGB.alpha());
                telemetry.addData("Red  ", sensorRGB.red());
                telemetry.addData("Green", sensorRGB.green());
                telemetry.addData("Blue ", sensorRGB.blue());
                telemetry.addData("Hue  ", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                ball_is_blue = sensorRGB.blue() < sensorRGB.red();

                if (vumark >= 0) {
                    state = 20;
                    start_time = getRuntime();
                }
            }

            if (state == 20) {
                //run(0.0, ball_is_blue ? 0.4 : -0.4, 0.0, 0.5, 0.5);
                servoPole_secdonary.setPosition(ball_is_blue ? 0.3 : 0.7);

                if (getRuntime() > start_time + 0.5) {
                    state = 30;
                    start_time = getRuntime();
                }
            }

            // Off board
            if (state == 30) {
                servoPole_secdonary.setPosition(0.5);
                servoPole.setPosition(0.3);

                run(0.0, 1.0, 0.0, 1.0, 1.0);

                if (getRuntime() > start_time + 0.8) {
                    state = 31;
                    start_time = getRuntime();
                }
            }

            // Turn
            if (state == 31) {
                run(0.0, 0.0, 1.0, 1.0, 1.0);

                if (getRuntime() > start_time + 0.75) {
                    state = 32;
                    start_time = getRuntime();
                }
            }

            // FWD
            if (state == 32) {
                run(0.0, -1.0, 0.0, 0.7, 1.0);

                if (getRuntime() > start_time + 0.35) {
                    state = 33;
                    start_time = getRuntime();
                }
            }

            // To Column
            if (state == 33) {
                run(-1.0, 0.0, 0.0, 1, 1.0);

                if (getRuntime() > start_time + (double)(vumark) * 0.25 +0.1) {
                    run(0.0, 0.0, 0.0, 0.5, 1.0);
                    state = 34;
                    start_time = getRuntime();
                }
            }

            // Offload
            if (state == 34) {
                servoOffload.setPosition(0.75);

                if (getRuntime() > start_time + 0.5) {
                    state = 35;
                    start_time = getRuntime();
                }
            }

            // Go back before punch it
            if (state == 34) {
                run(0.0, 1.0, 0.0, 1.0, 1.0);

                if (getRuntime() > start_time + 0.3) {
                    state = 35;
                    start_time = getRuntime();
                }
            }

            // Now PUNCH it
            if (state == 35) {
                run(0.0, -1.0, 0.0, 1.0, 1.0);

                if (getRuntime() > start_time + 0.4) {
                    state = 40;
                    start_time = getRuntime();
                }
            }

            // Prepare for rest
            if (state == 40) {
                intakeL.setPower(-0.45);
                intakeR.setPower(-0.45);

                run(0.0, 0.0, 0.0, 0.0, 0.0);

                if (getRuntime() > start_time + 1.0) {
                    intakeL.setPower(0.0);
                    intakeR.setPower(0.0);
                    state = 99;
                    start_time = getRuntime();
                }
            }

            // Playing with water
            if (state == 99) {
                leftFDrive.setPower(0);
                rightFDrive.setPower(0);
                leftBDrive.setPower(0);
                rightBDrive.setPower(0);
            }

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }

    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
