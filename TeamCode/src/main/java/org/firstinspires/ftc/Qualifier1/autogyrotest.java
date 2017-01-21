package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Shlok on 1/3/2017. *
 */

@Autonomous(name = "autogyrotest", group = "Final")
public class autogyrotest extends LinearOpMode {
    // Drive System for Basic Movement
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private DcMotor motorBP;
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;
    private int count = 0;

    // Eoncoder Setup
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1120 for andymarx, 1440 for tetrix
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        // Setting up Sensors

        //Color Sensor
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Range Sensor
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // Gyro Sensor
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        telemetry.addData(">", "GYRO CALIBRATING, PLEASE HOLD");
        telemetry.update();
        gyro.calibrate();
        while(gyro.isCalibrating()) { sleep(50); }
        telemetry.addData(">", "GYRO CALIBRATED, CLEARED FOR TAKEOFF");
        telemetry.update();

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

        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        heading = gyro.getHeading();
        angleZ  = gyro.getIntegratedZValue();

        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);

        // S1: Sideways 39 Inches with 10 Sec timeout


        // S3: Forward 17 Inches with 20 Sec timeout
        telemetry.addData("Path", "Driving Straight");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, -25, -25, -25, -25, 3);
        telemetry.addData("Path1",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());
        telemetry.update();

        /*// S3: Forward 17 Inches with 20 Sec timeout
        telemetry.addData("Path", "Rotating");
        telemetry.update();
        sleep(1000);
        encoderDrive(0.2, -59, 59, -59, 50, 7);
        telemetry.addData("Path1",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());
        telemetry.update();*/

        while(gyro.getHeading() < 270 || (gyro.getHeading() > 275) && opModeIsActive()) {
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.update();
            motorFR.setPower(-.15);
            motorBR.setPower(-.15);
            motorFL.setPower(.15);
            motorBL.setPower(.15);
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.update();
        }
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);

        count = 1;
        telemetry.addData("Path", "Driving Straight");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, -60, -60, -60, -60, 7);
        telemetry.addData("Path1",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());
        telemetry.update();

        /*while (rangeSensor.cmUltrasonic() > 15 || rangeSensor.cmUltrasonic() == 0 && opModeIsActive()) {
            motorFR.setPower(-0.5);
            motorFL.setPower(-0.5);
            motorBR.setPower(-0.5);
            motorBL.setPower(-0.5);
            telemetry.addData("cm sensed", rangeSensor.cmUltrasonic());
            telemetry.update();
        }
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);*/
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
                 /*   && ((motorFR.getCurrentPosition() <  newmotorFRTarget) && motorFR.isBusy())
                    || ((motorFL.getCurrentPosition() <  newmotorFLTarget) && motorFL.isBusy())
                    || ((motorBR.getCurrentPosition() <  newmotorBRTarget) && motorBR.isBusy())
                    || ((motorBL.getCurrentPosition() <  newmotorBLTarget) && motorBL.isBusy()))*/
                    && (motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy()))
            {
                if (count == 1 && rangeSensor.cmUltrasonic() < 20 && rangeSensor.cmUltrasonic() != 0)
                    break;
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