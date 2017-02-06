package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Shlok on 1/3/2017. *
 */

@Autonomous(name = "BallShootingStop", group = "Test")
public class BallShootingStop extends LinearOpMode {
    private ElapsedTime runtime2 = new ElapsedTime();

    // Drive System for Basic Movement
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private DcMotor motorBB;
    private DcMotor motorCC;
    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cGyro gyro;
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

        // Initialization of Motors
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBB = hardwareMap.dcMotor.get("motorBB");
        motorCC = hardwareMap.dcMotor.get("motorCC");
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

        telemetry.addData("Status", "Finished Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
       /* telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                motorFR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorBL.getCurrentPosition());
        telemetry.update();*/

        waitForStart();  // Everything after this point will only happen after play button pressed


        // Start of all movement
        sleep(10000);
        double start = runtime2.seconds();
        while(runtime2.seconds() < start + 1) {
            motorFR.setPower(-1);
            motorFL.setPower(-1);
            motorBR.setPower(-1);
            motorBL.setPower(-1);
        }
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);

        // Shoot the ball after movement
        while(runtime2.seconds() < start +  2) {
            motorCC.setPower(1);
        }
        motorCC.setPower(0);
        sleep(500);







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
                telemetry.addData("Heading", gyro.getHeading());
                telemetry.addData("cm", rangeSensor.cmUltrasonic());
                telemetry.addData("motorFR: ", motorFR.getPower());
                telemetry.addData("motorFL: ", motorFL.getPower());
                telemetry.addData("motorBR: ", motorBR.getPower());
                telemetry.addData("motorBL: ", motorBL.getPower());
                telemetry.addData("Red ", colorSensor.red());

                telemetry.update();
                if (count == 1 && rangeSensor.cmUltrasonic() < 20 && rangeSensor.cmUltrasonic() != 0 &&opModeIsActive())
                    break;

                if (count == 2 && colorSensor.red() > 1 && opModeIsActive())
                    break;

                if (count == 3 && rangeSensor.cmUltrasonic() < 10 && rangeSensor.cmUltrasonic() != 0 &&opModeIsActive())
                    break;


                /*if (!(count == 2 && gyro.getHeading() < 273 || (gyro.getHeading() > 278)) && opModeIsActive()) {
                    telemetry.addLine("Exiting");
                    break;
                }*/



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



