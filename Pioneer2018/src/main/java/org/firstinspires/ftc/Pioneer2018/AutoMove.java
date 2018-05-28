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

// VER 2


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
@Autonomous(name = "Move", group = "Auto")
//@Disabled
public class AutoMove extends LinearOpMode {

    ColorSensor sensorRGB;

    private Servo servoRight = null;
    private Servo servoLeft = null;
    private Servo servoPole = null;

    private DcMotor leftFDrive = null;
    private DcMotor rightFDrive = null;
    private DcMotor rightBDrive = null;
    private DcMotor leftBDrive = null;

    private DcMotor intakeL = null;
    private DcMotor intakeR = null;


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

    @Override
    public void runOpMode() {

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

        // wait for the start button to be pressed.
        waitForStart();

        servoRight.setPosition(0.0);
        servoLeft.setPosition(0.55);




        double start_time = getRuntime();
        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

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
                telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });



                state = 20;
                start_time = getRuntime();
            }

            if (state == 20) {
                double drivex = 0.0;
                double drivey = 0.0;

                double turn = 0.0;

                double velocity = 0.5;
                double ang_velocity = 0.5;

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

                if (getRuntime() > start_time + 0.5) {
                    state = 21;
                    start_time = getRuntime();
                }
            }

            if (state == 21) {
                double drivex = 0.0;
                double drivey = 0.0;

                double turn = 0.0;

                double velocity = 0.5;
                double ang_velocity = 0.5;

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

                if (getRuntime() > start_time + 3) {
                    state = 30;
                    start_time = getRuntime();
                    servoPole.setPosition(0);
                }
            }

            if (state == 30) {
                double drivex = -0.4;
                double drivey = -1.0;

                double turn = 0.0;

                double velocity = 1.0;
                double ang_velocity = 1.0;

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

                if (getRuntime() > start_time + 2.0) {
                    state = 40;
                    start_time = getRuntime();
                }
            }

            if (state == 40) {
                intakeL.setPower(-0.45);
                intakeR.setPower(-0.45);

                if (getRuntime() > start_time + 1.0) {
                    intakeL.setPower(0.0);
                    intakeR.setPower(0.0);
                    state = 99;
                    start_time = getRuntime();
                }
            }

            if (state == 99) {
                leftFDrive.setPower(0);
                rightFDrive.setPower(0);
                leftBDrive.setPower(0);
                rightBDrive.setPower(0);
            }

            telemetry.update();
        }

        // Set the panel stateback to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
