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

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "ColorTest", group = "Test")
public class ColorTest extends LinearOpMode {
  // For Line Follower
  OpticalDistanceSensor lightSensor; // ODS used for line follower
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
    lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_light");
    ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    telemetry.update();
    gyro.calibrate();
    while(gyro.isCalibrating()) { sleep(50); }
    telemetry.addData(">", "GYRO CALIBRATED, CLEARED FOR TAKEOFF");
    telemetry.update();

    // Reversing Motors
    motorFR.setDirection(DcMotor.Direction.REVERSE);
    motorBR.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();    // <-- DELETE IF UNNECESSARY



    // Color Sensor Stuff
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;
    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);




    // wait for the start button to be pressed.
    waitForStart();


    while (opModeIsActive()) {
      // Displays how long program has been running on the robot
      telemetry.addData("Runtime ", runtime.seconds());

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
     //Color Sensor Stuff
      // check the status of the x button on either gamepad.
      bCurrState = gamepad1.x;
      // check for button state transitions.
      if ((bCurrState == true) && (bCurrState != bPrevState))  {
        // button is transitioning to a pressed state. So Toggle LED
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
      }
      // update previous state variable.
      bPrevState = bCurrState;
      // convert the RGB values to HSV values.
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
      // send the info back to driver station using telemetry function.
      telemetry.addData("LED", bLedOn ? "On" : "Off");
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Hue", hsvValues[0]);
      telemetry.addData("Light Value: ", lightSensor.getLightDetected());

      // If Color Sensor senses blue, drive forward for one second
      /* if (colorSensor.blue() > 1) {
            double bluetime = runtime.seconds();
            while(runtime.seconds() < bluetime + 1) {
                motorFR.setPower(1);
                motorFL.setPower(1);
                motorBR.setPower(1);
                motorBL.setPower(1);
            }
        }*/


      if (lightSensor.getLightDetected() > 0.1) {
        telemetry.addData("SAW LINE", lightSensor.getLightDetected());

          /*double linetime = runtime.seconds();
        while (runtime.seconds() < linetime + 1) {
          // Get a correction
          correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected());

          // Sets the powers so they are no less than .075 and apply to correction
          if (correction <= 0) {
            leftPower = .075d - correction;
            lightPower = .075d;
          } else {
            leftPower = .075d;
            rightPower = .075d + correction;
          }

          // Sets the powers to the motors
          motorFL.setPower(leftPower);
          motorBL.setPower(leftPower);
          motorFR.setPower(rightPower);
          motorBR.setPower(rightPower);
        }*/
      }


      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      telemetry.update();
    }
  }
}


