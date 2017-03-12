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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Pushbot: Teleop POV", group = "Pushbot")
@Disabled
public class PushbotTeleopPOV_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;

        boolean apasare_b = false;
        float pozitiebs = -1 / 6;
        float pozitiebd = 1 / 6;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Andrei! Hello Robert!");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            //Activates gamepad2 for lifting the cap ball
            robot.liftMotor.setPower(-gamepad2.right_stick_y);
            robot.liftMotor.setPower(-gamepad2.right_stick_y);

            //servo matura (gheara) porneste pe pozitia MID_SERVO =  0.5
          /*  if(gamepad1.right_bumper){
               robot.matura.setPosition(1);
           }else if (gamepad1.left_bumper){
               robot.matura.setPosition(0.5);
            }*/

            /*if (gamepad1.left_bumper) {
                robot.liftClaw.setPosition(0);
                //de explicat
            } else if (gamepad1.right_bumper) {
                robot.liftClaw.setPosition(0.5);
                //de explicat
            }
*/
           /* if (gamepad1.b)
                apasare_b = !apasare_b;

            if (apasare_b) {
                robot.maturaDown.setPosition(1);
                robot.maturaUp.setPosition(1);
            } else {
                robot.maturaDown.setPosition(0.5);
                robot.maturaUp.setPosition(0.5);
            }

            if (gamepad1.left_trigger != 0) {
                robot.leftBeacon.setPosition(1);
            } else if (gamepad1.right_trigger != 0) {
                robot.rightBeacon.setPosition(0.5);
            }

*/
            // Activates the two brooms to collect the particles (Y) FORWARD and (A) REVERSE
         /*   if (gamepad2.y) {
                robot.collectMotor1.setPower(1.0);
                //robot.collectMotor2.setPower(1.0);
            } else if (gamepad2.a) {
                robot.collectMotor1.setPower(-1.0);
                //robot.collectMotor2.setPower(-1.0);
            } else {
                robot.collectMotor1.setPower(0);
                //robot.collectMotor2.setPower(0);
            }*/

      if (gamepad1.left_trigger!=0 || gamepad2.y) {
                robot.collectMotor1.setPower(1.0);
                //robot.collectMotor2.setPower(1.0);
            } else if (gamepad1.right_trigger!=0 || gamepad2.a) {
                robot.collectMotor1.setPower(-1.0);
                //robot.collectMotor2.setPower(-1.0);
            } else {
                robot.collectMotor1.setPower(0);
                //robot.collectMotor2.setPower(0);
            }


            // Move both servos to new position.  Assume servos are mirror image of each other.
            //   clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            //    robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            //    robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            //Activates the catapult when (X) is pressed
            if (gamepad1.x || gamepad2.x)
                robot.armMotor.setPower(robot.ARM_UP_POWER);
            else
                robot.armMotor.setPower(0.0);



            // Send telemetry message to signify robot running;
            telemetry.addData("ArmMotor", "" + getRuntime());
            telemetry.addData("claw", "Offset = %.2f", clawOffset);
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("lift motor in sus", "%.2f", robot.ARM_UP_POWER);
            telemetry.addData("arm motor: ", "Value:" + robot.leftMotor.getCurrentPosition());

            telemetry.update();


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}

