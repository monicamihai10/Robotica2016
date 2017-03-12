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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@Disabled
public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();// Use a Pushbot's hardware

    //SensorMRRangeSensor range= new SensorMRRangeSensor();
    //public SensorMRGyro gyro = new SensorMRGyro();

    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cRangeSensor rangeSensor;

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //gyro.init(hardwareMap);
        //range.init();

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED*0.4, 12.566, 12.566, 3.0);

        //gyro.turnAbsolute(45);

        /*runtime.reset();// S1: Forward 47 Inches with 5 Sec timeout
        trage(1, (-2880), 2.0);
        runtime.reset();
        //pt colectare prevedem un servomotor pt inlocuirea maturii
        colectare(1);
        runtime.reset();
        sleep(500);
        trage(1, (-2880), 2.0);*/

       // encoderTurnLeft(TURN_SPEED, 18, 4.0);// S2: Turn Right 12 Inches with 4 Sec timeout

        encoderSpinLeft(DRIVE_SPEED*0.7,21,-21,4.0); // aici ar trebui sa verificam cu gyro unghiul fata de pozitia initiala

        encoderDrive(DRIVE_SPEED*0.4, 37, 37, 3.0); // aici ar trebui sa verificam cu senzorul de distanta

        telemetry.addData("1","rangeCheckOptic",rangeCheckOptic()) ;
        telemetry.addData("2","rangeCheckSonic",rangeCheckSonic()) ;
        sleep(2000);

        encoderSpinLeft(DRIVE_SPEED*0.7,30,-30,4.0);

        encoderDrive(DRIVE_SPEED*0.4, -12, -12, 3.0);



        //encoderDrive(DRIVE_SPEED, 35, -35, 4.0);// S3: Reverse 24 Inches with 4 Sec timeout
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        //aplauze!!!

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        sleep(2000);     // pause for servos to move

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
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() || robot.rightMotor.isBusy())
                    && (robot.leftMotor.getCurrentPosition() <= newLeftTarget
                    || robot.rightMotor.getCurrentPosition() <= newRightTarget)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }


    public void trage(double speed, double leftInches, double timeoutS) {

        int newLeftTarget;
        int newRightTarget;
        float hsvValuesA[] = {0F, 0F, 0F};
        float hsvValuesB[] = {0F, 0F, 0F};

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.armMotor.getCurrentPosition() + ((int) leftInches);
            // newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.armMotor.setTargetPosition(newLeftTarget);
            //robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.armMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            //while (opModeIsActive() && robot.leftMotor.getCurrentPosition() < newLeftTarget || robot.rightMotor.getCurrentPosition() < -newRightTarget)
            while (opModeIsActive() && robot.armMotor.isBusy() &&
                    (runtime.seconds() < timeoutS)) {
                telemetry.addData("Left motor: ", "Value:" + robot.armMotor.getCurrentPosition());
                //telemetry.addData("Right motor: ","Value:" +robot.leftMotor.getCurrentPosition());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newLeftTarget);
                telemetry.addData("Path2", "Running at %7d",
                        robot.armMotor.getCurrentPosition());
                // robot.rightMotor.getCurrentPosition());

                //    telemetry.addData("Hue", hsvValuesB[0]);

                //End color sensor
                telemetry.update();
            }

            // Stop all motion;
            robot.armMotor.setPower(0);
            //robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }

    }

    public void colectare(double speed) {

        while (opModeIsActive() && runtime.seconds() < 2) {

            robot.collectMotor1.setPower(-speed);
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());

        }
        robot.collectMotor1.setPower(0);
        runtime.reset();
    }


    public void encoderTurnLeft (double speed,
                             double leftInches,
                             double timeoutS) {
        int newLeftTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            robot.leftMotor.setTargetPosition(newLeftTarget);


            // Turn On RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy())
                    && (robot.leftMotor.getCurrentPosition() <= newLeftTarget
                    )) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newLeftTarget);
                telemetry.addData("Path2", "Running at %7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderTurnRight(double speed,
                                 double rightInches,
                             double timeoutS) {

        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ( robot.rightMotor.isBusy())
                    && (robot.rightMotor.getCurrentPosition() <= newRightTarget)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newRightTarget);
                telemetry.addData("Path2", "Running at %7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderSpinRight (double speed, //pe dreapta pozitiv pe stanga negativ
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(-Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() || robot.rightMotor.isBusy())
                    && (robot.leftMotor.getCurrentPosition() >= newLeftTarget
                    || robot.rightMotor.getCurrentPosition() <= newRightTarget)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderSpinLeft(double speed,//pe stanga pozitiv pe dreapta negativ
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() || robot.rightMotor.isBusy())
                    && (robot.leftMotor.getCurrentPosition() <= newLeftTarget
                    || robot.rightMotor.getCurrentPosition() >= newRightTarget)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }


    public int rangeCheckSonic (){

        int distantaUltraSonica = rangeSensor.rawUltrasonic();

        return distantaUltraSonica;
    }

    public int rangeCheckOptic (){

        int distantaOptica = rangeSensor.rawOptical();

        return distantaOptica;

    }
}

