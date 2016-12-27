package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Shlok on 11/13/2016.
 */
@TeleOp(name = "TestTeleOp", group = "Test")
public class TestTeleOp extends LinearOpMode
{
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive())
        {
            motorFR.setPower(-gamepad1.left_stick_y);
            motorFL.setPower(-gamepad1.right_stick_y);
            idle();
        }
    }
}

