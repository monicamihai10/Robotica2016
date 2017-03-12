/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "sensor_ods".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR ODS", group = "Sensor")
@Disabled
public class SensorMROpticalDistance extends LinearOpMode {

  HardwarePushbot robot = new HardwarePushbot();

  //Instance of OpticalDistanceSensor
  OpticalDistanceSensor ODS;

  //Motors

  //Raw value is between 0 and 1
  double odsReadingRaw;

  // odsReadingRaw to the power of (-0.5)
  static double odsReadingLinear;

  @Override
  public void runOpMode() throws InterruptedException {

    //identify the port of the ODS and motors in the configuration file
    ODS = hardwareMap.opticalDistanceSensor.get("sensor_ods");
      robot.init(hardwareMap);



      //This program was designed around a robot that uses two gears on each side of the drive train.
    //If your robot uses direct drive between the motor and wheels or there are an odd number of gears, the opposite motor will need to be reversed.
    robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);

    robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    while (opModeIsActive()) {

      odsReadingRaw = ODS.getRawLightDetected() / 5;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
      odsReadingLinear = Math.pow(odsReadingRaw, 0.5);                //calculate linear value

      //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
      //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
      //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
      //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.
      robot.leftMotor.setPower(odsReadingLinear * 2);
      robot.rightMotor.setPower(0.5 - (odsReadingLinear * 2));

      telemetry.addData("0 ODS Raw", odsReadingRaw);
      telemetry.addData("1 ODS linear", odsReadingLinear);
      telemetry.addData("2 Motor Left", robot.leftMotor.getPower());
      telemetry.addData("3 Motor Right", robot.rightMotor.getPower());
      telemetry.update();
    }
  }
}
