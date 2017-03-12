/*
Copyright (c) 2016 Robert Atkinson

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
package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code assumes that you do NOT have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * The desired path in this example is:
 * - Drive forward for 3 seconds
 * - Spin right for 1.3 seconds
 * - Drive Backwards for 1 Second
 * - Stop and close the claw.
 * <p>
 * The code is written in a simple form with no optimizations.
 * However, there are several ways that this type of sequence could be streamlined,
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Time", group="Pushbot")
@Disabled
public class PushbotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    ColorSensor cs;

    TouchSensor ts;

    ModernRoboticsI2cRangeSensor rs;

    DeviceInterfaceModule CDI;



    @Override
    public void runOpMode() {

        boolean touchState=false;
        boolean LEDState=true;

        cs =  hardwareMap.colorSensor.get("sensor_color");

        rs = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        ts = hardwareMap.touchSensor.get("sensor_touch");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        float hsvValues [] = {0, 0, 0};

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            /*if (ts.isPressed()) {

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }*/
            
            }

        // Step 2:  Spin right for 1.3 seconds
        robot.leftMotor.setPower(TURN_SPEED);
        robot.rightMotor.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            if (ts.isPressed()) {

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
        }

        // Step 3:  Drive Backwards for 1 Second
        robot.leftMotor.setPower(-FORWARD_SPEED);
        robot.rightMotor.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();




                cs.enableLed(true);
                Color.RGBToHSV(cs.red() * 8, cs.green() * 8, cs.blue() * 8, hsvValues);}


                telemetry.addData("Clear", cs.alpha());
                telemetry.addData("Red  ", cs.red());
                telemetry.addData("Green", cs.green());
                telemetry.addData("Blue ", cs.blue());
                telemetry.addData("Hue", hsvValues[0]);

                telemetry.update();




        // Step 4:  Stop and close the claw.
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}