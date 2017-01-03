package FTC6127.Qualifier1;

/**
 * Created by Shruthi Ramalingam on 12/30/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name = "Autonomous6127", group = "Test")

 public class Autonomous6127 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushBot robot   = new HardwarePushBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
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
        telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        encoderDrive(DRIVE_SPEED,  -30, -30, 30, 30, 3.0); // S1: Forward 47 Inches with 5 Sec timeout
        telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());


        telemetry.addData("Path", "Straight");
        telemetry.update();
        sleep(1000);
        encoderDrive(TURN_SPEED,   -30, 30, -30, 30, 3.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());


        telemetry.addData("Path", "Turn Right");
        telemetry.update();
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 30, 30, 30, 30, 3.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                robot.topleftMotor.getCurrentPosition(),
                robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());

        telemetry.addData("Path", "Reverse");
        telemetry.update();

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
                             double timeoutS) {
        int newTopLeftTarget;
        int newTopRightTarget;
        int newBotLeftTarget;
        int newBotRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTopLeftTarget = robot.topleftMotor.getCurrentPosition() + (int)(topleftInches * COUNTS_PER_INCH);
            newTopRightTarget = robot.toprightMotor.getCurrentPosition() + (int)(toprightInches * COUNTS_PER_INCH);
            newBotLeftTarget = robot.topleftMotor.getCurrentPosition() + (int)(botleftInches * COUNTS_PER_INCH);
            newBotRightTarget = robot.toprightMotor.getCurrentPosition() + (int)(botrightInches * COUNTS_PER_INCH);
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
            robot.topleftMotor.setPower(Math.abs(speed));
            robot.toprightMotor.setPower(Math.abs(speed));

            robot.botleftMotor.setPower(Math.abs(speed));
            robot.botrightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.topleftMotor.getCurrentPosition() <  newTopLeftTarget && robot.toprightMotor.isBusy()))
            {
                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newTopLeftTarget,  newTopRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                //        robot.topleftMotor.getCurrentPosition(),
                //        robot.toprightMotor.getCurrentPosition());
               // telemetry.update();
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

            telemetry.addData("Path0",  "Starting at %7d :%7d  %7d :%7d ",
                    robot.topleftMotor.getCurrentPosition(),
                    robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());

        }
    }

}

