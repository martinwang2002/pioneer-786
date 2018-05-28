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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop Manual Test2", group="Testing Opmode")
//@Disabled
public class TeleopMain2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFDrive = null;
    private DcMotor rightFDrive = null;
    private DcMotor rightBDrive = null;
    private DcMotor leftBDrive = null;
    private DcMotor intakeL = null;
    private DcMotor intakeR = null;
    private DcMotor lifting = null;
    private boolean lifting_fwd = true;
    private Servo servoTest = null;
    private Servo servoRight = null;
    private Servo servoLeft = null;
    private Servo servoL = null;
    private Servo servoR = null;
    private Servo servoPole = null;

    private boolean intakeDown = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBDrive = hardwareMap.get(DcMotor.class, "rb");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");

        lifting = hardwareMap.get(DcMotor.class, "lifting");
        lifting.setDirection(DcMotor.Direction.FORWARD);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.FORWARD);
        intakeR.setDirection(DcMotor.Direction.REVERSE);
        // Initialize the hardware variables.
        servoTest = hardwareMap.get(Servo.class, "platform_servo");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        servoPole = hardwareMap.get(Servo.class, "servoPole");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        servoPole.setPosition(0.4);

        // =========================================================================================
        // Motors
        // =========================================================================================

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFPower;
        double leftBPower;
        double rightFPower;
        double rightBPower;

        double drivex = -gamepad1.left_stick_x;
        double drivey = gamepad1.left_stick_y;

        double turn = gamepad1.right_stick_x;

        double velocity = (gamepad1.left_stick_button ? 1.0 : 0.75) * (1.0 - gamepad1.left_trigger);
        double ang_velocity = (gamepad1.right_stick_button ? 1.0 : 0.75) * (1.0 - gamepad1.left_trigger);

        velocity *= gamepad1.left_bumper ? 0.5 : 1.0;

        double[] v0 = MechanismUtil.calcv(turn * ang_velocity, drivex, drivey);

        rightFPower = Range.clip(v0[0] * velocity, -1.0, 1.0);
        leftFPower = Range.clip(v0[1] * velocity, -1.0, 1.0);
        leftBPower = Range.clip(v0[2] * velocity, -1.0, 1.0);
        rightBPower = Range.clip(v0[3] * velocity, -1.0, 1.0);

        // Send calculated power to wheels
        leftFDrive.setPower(Math.abs(leftFPower) * leftFPower);
        rightFDrive.setPower(Math.abs(rightFPower) * rightFPower);
        leftBDrive.setPower(Math.abs(leftBPower) * leftBPower);
        rightBDrive.setPower(Math.abs(rightBPower) * rightBPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)", v0[0], v0[1], v0[2], v0[3]);

        // =========================================================================================
        // Offload platform
        // =========================================================================================
        if(gamepad1.b){
            //move to 0 degrees.
            servoTest.setPosition(0.75);
        } else {
            //move to 90 degrees.
            servoTest.setPosition(0.1);
        }

        // =========================================================================================
        // Intake Position
        // =========================================================================================
        if(gamepad1.x && gamepad1.y) {
            intakeDown = !intakeDown;
            telemetry.addData("Intake Status", intakeDown ? "down" : "up");
        }
        if (intakeDown) {
            servoRight.setPosition(0);
            servoLeft.setPosition(0.3);
        } else {
            servoRight.setPosition(0.5);
            servoLeft.setPosition(0.8);
        }


        // =========================================================================================
        // Intake (With Diffrential Speed)
        // =========================================================================================
        boolean intake_activate = gamepad1.right_bumper;

        double intake_negate = gamepad1.left_bumper ? -1.0 : 1.0;

        if(gamepad1.x){
            intakeL.setPower(intake_activate ? intake_negate * 0.25 : 0.0);
            intakeR.setPower(intake_activate ? intake_negate * 1.0  : 0.0);
        }
        if(gamepad1.y){
            intakeL.setPower(intake_activate ? intake_negate * 1.0  : 0.0);
            intakeR.setPower(intake_activate ? intake_negate * 0.25 : 0.0);
        }
        if(!(gamepad1.x || gamepad1.y)){
            intakeL.setPower(intake_activate ? intake_negate * 1.0  : 0.0);
            intakeR.setPower(intake_activate ? intake_negate * 1.0  : 0.0);
        }

        // =========================================================================================
        // Feeder
        // =========================================================================================
        if(gamepad1.a){
            servoL.setPosition(0);
            servoR.setPosition(1);
        }
        else{
            servoL.setPosition(0.54);
            servoR.setPosition(0.46);
        }

        // =========================================================================================
        // Lifting platform
        // =========================================================================================
        if (gamepad1.dpad_up) {
            if (!lifting_fwd) {
                lifting.setDirection(DcMotor.Direction.FORWARD);
                lifting_fwd = true;
            }
            lifting.setPower(1.0);
            telemetry.addData("Lifting pwr", 1.0);
        }
        else if (gamepad1.dpad_down) {
            if (lifting_fwd) {
                lifting.setDirection(DcMotor.Direction.REVERSE);
                lifting_fwd = false;
            }
            lifting.setPower(1.0);
            telemetry.addData("Lifting pwr", -1.0);
        }
        else {
            lifting.setPower(0.0);
        }


    }


    /*
     * Code to run ONCE after the driver hits STOPls
     *
     */
    @Override
    public void stop() {
    }

}
