package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Shlok on 1/22/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpShootingForwardDrive", group = "Final")
public class TeleOpShootingForwardDrive extends LinearOpMode
{
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private DcMotor motorCC;
    private DcMotor motorBB;

    public static final double ARM_UP_POWER    =  0.45 ;
    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorCC = hardwareMap.dcMotor.get("motorCC");
        motorBB = hardwareMap.dcMotor.get("motorBB");
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorCC.setDirection(DcMotor.Direction.FORWARD);
        motorBB.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorCC.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {
           /* float FRspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
            float BRspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
            float FLspeed = gamepad1.right_stick_y + gamepad1.right_stick_x;
            float BLspeed = gamepad1.right_stick_y - gamepad1.right_stick_x;*/

            float FLspeed = gamepad1.right_stick_y - gamepad1.right_stick_x;
            float BLspeed = gamepad1.right_stick_y + gamepad1.right_stick_x;
            float FRspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
            float BRspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;

            FRspeed = Range.clip(FRspeed, -1, 1);
            FLspeed = Range.clip(FLspeed, -1, 1);
            BRspeed = Range.clip(BRspeed, -1, 1);
            BLspeed = Range.clip(BLspeed, -1, 1);

            motorFR.setPower(-FRspeed);
            motorFL.setPower(-FLspeed);
            motorBR.setPower(-BRspeed);
            motorBL.setPower(-BLspeed);

            if (gamepad1.y)
            {
                //set choo choo encoder position
                motorCC.setTargetPosition(1080);
                motorCC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorCC.setPower(0.5);
                telemetry.addData("position: ", motorCC.getCurrentPosition());
            }
            else {
                motorCC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorCC.setPower(0);
            }

            if (gamepad1.x)
            {
                //set choo choo encoder position
                motorBB.setPower(0.5);
                telemetry.addData("BB",  "%.2f", 0.5);
                telemetry.update();

            }
            else {

                motorBB.setPower(0);
            }
        }
    }
}