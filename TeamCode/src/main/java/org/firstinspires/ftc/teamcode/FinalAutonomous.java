package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Shlok on 1/3/2017.
 *
 */

@Autonomous(name = "FinalAutonomous", group = "Ready")
public class FinalAutonomous extends LinearOpMode {
    // Drive System for Basic Movement
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    // Timer
    private ElapsedTime     runtime = new ElapsedTime();
    // Eoncoder Setup
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1120 for andymarx, 1440 for tetrix
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode(){
        // Initialization of Motors
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());
        telemetry.update();

        waitForStart();

        // S3: Reverse 17 Inches with 20 Sec timeout
        telemetry.addData("Path", "Driving Straight");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 18, 18, 18, 18, 3);
        telemetry.addData("Path1",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());

        // S3: Sideways 39 Inches with 10 Sec timeout
        telemetry.addData("Path", "Driving Sideways");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 40, -40, -40, 40, 10);
        telemetry.addData("Path2",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());
        telemetry.addData("Path", "Complete");
        telemetry.update();

        /**
         * To Be Added:
         *  1. Range Sensor senses wall and stops robot when it's near the wall and a count variable
         *  increments past one, nullifying the if statement with the range sensor inside of the
         *  code dictating what is done during encoderDrive
         *  Helper Programs: RangeTest(not made yet), GyroTest, MRI_ODS_Wall_Follow
         *
         *  2. New count type variable made for the color sensor portion that allows the if statement
         *  to remain active when the count is less than 2 (for the two beacons). Robot slowly strafes
         *  until it reaches a beacon with the corresponding color and when it reaches the color
         *  it presses button and then backs out and repeats for the next beacon.
         *  Helper Programs: ColorTest, EncoderTest
         *
         *  3. Strafe away from the wall, turn, and position the robot so it can shoot the ball
         *  Helper Programs: EncoderTest, GyroTest
         *
         *  4. Hit the ball of the wooden stand
         *  Helper Programs: EncoderTest, GyroTest
         *
         *  Notes: Add some part of the program so that the robot stays a certain distance from the
         *  wall at all times
         *  Helper Programs: EncoderTest, RangeTest(not made yet), MRI_ODS_Wall_Follow
         *
         */

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double motorFRInches, double motorFLInches, double motorBRInches, double motorBLInches,
                             double timeoutS) {
        int newmotorFRTarget;
        int newmotorFLTarget;
        int newmotorBRTarget;
        int newmotorBLTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newmotorFRTarget = motorFR.getCurrentPosition() + (int)(motorFRInches * COUNTS_PER_INCH);
            newmotorFLTarget = motorFL.getCurrentPosition() + (int)(motorFLInches * COUNTS_PER_INCH);
            newmotorBRTarget = motorBR.getCurrentPosition() + (int)(motorBRInches * COUNTS_PER_INCH);
            newmotorBLTarget = motorBL.getCurrentPosition() + (int)(motorBLInches * COUNTS_PER_INCH);
            motorFR.setTargetPosition(newmotorFRTarget);
            motorFL.setTargetPosition(newmotorFLTarget);
            motorBR.setTargetPosition(newmotorBRTarget);
            motorBL.setTargetPosition(newmotorBLTarget);


            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS)
                    && ((motorFR.getCurrentPosition() <  newmotorFRTarget) && motorFR.isBusy())
                    || ((motorFL.getCurrentPosition() <  newmotorFLTarget) && motorFL.isBusy())
                    || ((motorBR.getCurrentPosition() <  newmotorBRTarget) && motorBR.isBusy())
                    || ((motorBL.getCurrentPosition() <  newmotorBLTarget) && motorBL.isBusy()))
            {
                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget,  newmotorFRTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                //        motorFL.getCurrentPosition(),
                //        motorFR.getCurrentPosition());
                // telemetry.update();
            }

            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move

            telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                    motorFR.getCurrentPosition(),
                    motorFL.getCurrentPosition(),
                    motorBR.getCurrentPosition(),
                    motorBL.getCurrentPosition());

        }
    }

}



