/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.Qualifier1;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Shruthi Ramalingam on 01/06/2016.
 */

@Autonomous(name="Autonomous6127WithGyro", group="Pushbot")
//@Disabled
public class Autonomous6127WithGyro extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushBot         robot   = new HardwarePushBot();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /

                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable



    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.topleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.toprightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        robot.topleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.toprightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
      //  telemetry.addData("Path", "Drive Straight");
      //  telemetry.update();
      //      gyroDrive(DRIVE_SPEED, 43.0, 0.0);    // Drive FWD
        // 48 inches
        telemetry.addData("Path", "Turn Left");
        telemetry.update();
            gyroTurn( TURN_SPEED, 45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        telemetry.addData("Path", "Hold");
        telemetry.update();
      //  gyroHold( TURN_SPEED,  0.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        telemetry.addData("Path", "Drive Straight");
        telemetry.update();
        gyroDrive( DRIVE_SPEED, 45.0, 0.0);    // Drive FWD 15 inches
       // gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
       // gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
       // gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
       // gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
       // gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int newTopLeftTarget;
        int newTopRightTarget;
        int newBotLeftTarget;
        int newBotRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newTopLeftTarget = robot.topleftMotor.getCurrentPosition() + moveCounts;
            newTopRightTarget = robot.toprightMotor.getCurrentPosition() + moveCounts;
            newBotLeftTarget = robot.botleftMotor.getCurrentPosition() + moveCounts;
            newBotRightTarget = robot.botrightMotor.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            robot.topleftMotor.setTargetPosition(newTopLeftTarget);
            robot.toprightMotor.setTargetPosition(newTopRightTarget);
            robot.botleftMotor.setTargetPosition(newBotLeftTarget);
            robot.botrightMotor.setTargetPosition(newBotRightTarget);

            robot.topleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.toprightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            robot.topleftMotor.setPower(Math.abs(speed));
            robot.botleftMotor.setPower(Math.abs(speed));
            robot.toprightMotor.setPower(Math.abs(speed));
            robot.botrightMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.topleftMotor.isBusy() && robot.toprightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }


                robot.toprightMotor.setPower(rightSpeed);
                robot.topleftMotor.setPower(leftSpeed);
                robot.botleftMotor.setPower(leftSpeed);
                robot.botrightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "Starting at %7d :%7d  %7d :%7d ",      newTopLeftTarget,  newTopRightTarget,newBotLeftTarget,newBotRightTarget);
                telemetry.addData("Path",  "Starting at %7d :%7d  %7d :%7d ",
                        robot.topleftMotor.getCurrentPosition(),
                        robot.toprightMotor.getCurrentPosition(),  robot.botleftMotor.getCurrentPosition(),  robot.botrightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.toprightMotor.setPower(0);
            robot.topleftMotor.setPower(0);
            robot.botleftMotor.setPower(0);
            robot.botrightMotor.setPower(0);

            // Turn off RUN_TO_POSITION

            robot.topleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.toprightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.botleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.botrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.toprightMotor.setPower(0);
        robot.topleftMotor.setPower(0);
        robot.botleftMotor.setPower(0);
        robot.botrightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.topleftMotor.setPower(leftSpeed);
        robot.botleftMotor.setPower(leftSpeed);
        robot.toprightMotor.setPower(rightSpeed);
        robot.botrightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
