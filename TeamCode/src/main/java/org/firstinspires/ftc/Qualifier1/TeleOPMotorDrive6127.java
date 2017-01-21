package FTC6127.Qualifier1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC6127 on 12/26/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

    @TeleOp(name = "TeleOPMotorDrive6127", group = "Test")
    public class TeleOPMotorDrive6127 extends LinearOpMode
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
                float FLspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
                float BLspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
                float FRspeed = gamepad1.right_stick_y + gamepad1.right_stick_x;
                float BRspeed = gamepad1.right_stick_y - gamepad1.right_stick_x;

                FRspeed = Range.clip(FRspeed, -1, 1);
                FLspeed = Range.clip(FLspeed, -1, 1);
                BRspeed = Range.clip(BRspeed, -1, 1);
                BLspeed = Range.clip(BLspeed, -1, 1);

                motorFR.setPower(FRspeed);
                motorFL.setPower(FLspeed);
                motorBR.setPower(BRspeed);
                motorBL.setPower(BLspeed);


            }
        }
    }



