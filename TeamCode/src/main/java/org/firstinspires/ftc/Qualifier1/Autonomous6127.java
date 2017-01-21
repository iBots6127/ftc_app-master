package org.firstinspires.ftc.Qualifier1;

/**
 * Created by Shruthi Ramalingam on 12/30/2016.
 */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name = "Autonomous6127", group = "Test")

 public class Autonomous6127 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushBot robot   = new HardwarePushBot();   // Use a Pushbot's hardware
    ModernRoboticsI2cRangeSensor rangeSensor    = null;
    ColorSensor colorSensor;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.7;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        colorSensor.enableLed(false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.topleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.toprightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        robot.topleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.toprightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
        telemetry.update();


        // Color Sensor Stuff
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        telemetry.addData("Path", "Straight friday");
        telemetry.update();
<<<<<<< HEAD
        encoderDrive(DRIVE_SPEED,  48, 48, 48, 48, 2.0 , false); // S1: Forward 47 Inches with 5 Sec timeout
=======
        encoderDrive(DRIVE_SPEED,  65, 65, 65, 65, 2.5 , false); // S1: Forward 47 Inches with 5 Sec timeout
        telemetry.addData("final",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
>>>>>>> refs/remotes/origin/master


        telemetry.addData("Path", "Turn Left");
        telemetry.update();
        sleep(1000);
<<<<<<< HEAD
        encoderDrive(TURN_SPEED,   34, 34, 34, 34, 2.8, true);  // S2: Turn Left with 4 Sec timeout
=======
        encoderDrive(TURN_SPEED,   -15, 15, -15, 15, 2.7, false);  // S2: Turn Left with 4 Sec timeout
        telemetry.addData("final",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
>>>>>>> refs/remotes/origin/master


        telemetry.addData("Path", "Straight2");
        telemetry.update();
        sleep(1000);
<<<<<<< HEAD
        encoderDrive(DRIVE_SPEED, 32, 32, 32, 32, 1.93,false);  // S3: Reverse 24 Inches with 4 Sec timeout
=======
        encoderDrive(DRIVE_SPEED, 48, 48, 48, 48, 3 ,false);  // S3: Reverse 24 Inches with 4 Sec timeout
        telemetry.addData("final",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());

        telemetry.addData("Path", "Glide Right");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 40, -40, -40, 40, 3.0, false);  // S3: Reverse 24 Inches with 4 Sec timeout
        telemetry.addData("final",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
>>>>>>> refs/remotes/origin/master

        sleep(2000);

        // read the color sensor value here

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);

<<<<<<< HEAD
        if(colorSensor.red() >= 2)
        {
            encoderDrive(DRIVE_SPEED,  2, 2, 2, 2, 2 , false); // S1: Forward 47 Inches with 5 Sec timeout
        }
        else
        {
            encoderDrive(DRIVE_SPEED, -7.5, 7.5, 7.5, -7.5, .36, false); // S1: Forward 47 Inches with 5 Sec timeout

            {
                encoderDrive(DRIVE_SPEED, 4, 4, 4, 4, .5, false); // S1: Forward 47 Inches with 5 Sec timeout
            }
        }




        if(colorSensor.blue() > 1)
        {
            encoderDrive(DRIVE_SPEED,  2, 2, 2, 2, 2 , false); // S1: Forward 47 Inches with 5 Sec timeout

        }
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
=======
>>>>>>> refs/remotes/origin/master

        telemetry.update();
        sleep(3000);
      //  printrangesensorvalues();

 //       telemetry.addData("Path", "Glide Right");
 //       telemetry.update();
 //       sleep(1000);
 //        encoderDrive(DRIVE_SPEED, 28, -28, -28, 28, 3.0, false);  // S3: Reverse 24 Inches with 4 Sec timeout



        sleep(1000);     // pause for servos to move



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
                             double topleftInches, double toprightInches, double botleftInches, double botrightInches,
                             double timeoutS,boolean turn) {
        int newTopLeftTarget;
        int newTopRightTarget;
        int newBotLeftTarget;
        int newBotRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTopLeftTarget = robot.topleftMotor.getCurrentPosition() + (int)(topleftInches * COUNTS_PER_INCH);
            newTopRightTarget = robot.toprightMotor.getCurrentPosition() + (int)(toprightInches * COUNTS_PER_INCH);
            newBotLeftTarget = robot.botleftMotor.getCurrentPosition() + (int)(botleftInches * COUNTS_PER_INCH);
            newBotRightTarget = robot.botrightMotor.getCurrentPosition() + (int)(botrightInches * COUNTS_PER_INCH);
            robot.topleftMotor.setTargetPosition(newTopLeftTarget);
            robot.toprightMotor.setTargetPosition(newTopRightTarget);
            robot.botleftMotor.setTargetPosition(newBotLeftTarget);
            robot.botrightMotor.setTargetPosition(newBotRightTarget);


            // Turn On RUN_TO_POSITION
            robot.topleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.toprightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            if (turn == true)
            {
                robot.topleftMotor.setPower(0);
                robot.botleftMotor.setPower(0);
            }
            else {
                robot.topleftMotor.setPower(Math.abs(speed));
                robot.botleftMotor.setPower(Math.abs(speed));

            }
            robot.toprightMotor.setPower(Math.abs(speed));
            robot.botrightMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS)  && robot.toprightMotor.isBusy())
            {
                // Display it for the driver.
             //   telemetry.addData("Path1",  "Running to %7d :%7d", newTopLeftTarget,  newTopRightTarget);
                telemetry.addData("Path",  "Starting at %7d :%7d  %7d :%7d ",
                        robot.topleftMotor.getCurrentPosition(),
                        robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
                telemetry.addData("runtime",  "time at %7f",runtime.seconds());

                telemetry.update();
            }

            // Stop all motion;
            robot.topleftMotor.setPower(0);
            robot.toprightMotor.setPower(0);
            robot.botleftMotor.setPower(0);
            robot.botrightMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.topleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.toprightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.botleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.botrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move


        }
    }


    public void printrangesensorvalues()
    {

        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

    }


}

