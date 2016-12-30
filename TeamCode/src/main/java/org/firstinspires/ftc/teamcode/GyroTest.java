/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "GyroTest", group = "Test")
public class GyroTest extends LinearOpMode {
    double rightPower, leftPower, correction, lightPower;
    final double PERFECT_COLOR_VALUE = 0.2;
    // Color Sensing
    ColorSensor colorSensor;
    // Basic Movement
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Set references to objects
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;


        telemetry.addData(">", "GYRO CALIBRATING, PLEASE HOLD");
        telemetry.update();
        gyro.calibrate();
        while(gyro.isCalibrating()) { sleep(50); }
        telemetry.addData(">", "GYRO CALIBRATED, CLEARED FOR TAKEOFF");
        telemetry.update();

        // Reversing Motors
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        // wait for the start button to be pressed.
        waitForStart();

        while(gyro.getHeading() >= 90) {
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.update();
            motorFR.setPower(.5);
            motorBR.setPower(.5);
            motorFL.setPower(-.5);
            motorBL.setPower(-.5);
        }


        double start = runtime.seconds();
        while(runtime.seconds() < start + 1) {
            motorFR.setPower(.5);
            motorBR.setPower(.5);
            motorFL.setPower(.5);
            motorBL.setPower(.5);
        }
        /*
        while (opModeIsActive()) {
            // Displays how long program has been running on the robot

            // Mecanum Drive (From TestMecanum)
            float FLspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
            float BLspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
            float FRspeed = gamepad1.right_stick_y + gamepad1.right_stick_x;
            float BRspeed = gamepad1.right_stick_y - gamepad1.right_stick_x;
            //Clips speed ranges from -1 to 1
            FRspeed = Range.clip(FRspeed, -1, 1);
            FLspeed = Range.clip(FLspeed, -1, 1);
            BRspeed = Range.clip(BRspeed, -1, 1);
            BLspeed = Range.clip(BLspeed, -1, 1);
            // Sets motor powers
            motorFR.setPower(FRspeed);
            motorFL.setPower(FLspeed);
            motorBR.setPower(BRspeed);
            motorBL.setPower(BLspeed);
            // Slows Down Movement
            boolean bumperHeld = gamepad1.x;
            if (bumperHeld) {
                motorFR.setPower(FRspeed/2);
                motorFL.setPower(FLspeed/2);
                motorBR.setPower(BRspeed/2);
                motorBL.setPower(BLspeed/2);
            }
            // get the x, y, and z values (rate of change of angle).
            xVal = gyro.rawX();
            yVal = gyro.rawY();
            zVal = gyro.rawZ();

            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            heading = gyro.getHeading();
            angleZ  = gyro.getIntegratedZValue();

            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);

            telemetry.update();
        }
        */
    }
}

// 1120
