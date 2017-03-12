package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Mosu on 23.02.2017.
 */
public class MosuActionPlan extends LinearOpMode {


    //Ce avem de facut:
    //

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 560;    // era 720 valoarea e 140 PPR la Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.4;
    //    ColorSensor colorSensorA ;
//    ColorSensor colorSensorB ;
    ColorSensorWrapper sensor_colorA;
    ColorSensorWrapper sensor_colorB;


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        sensor_colorA = new ColorSensorWrapper("sensor_colorA", hardwareMap, null, telemetry);
        sensor_colorB = new ColorSensorWrapper("sensor_colorB", hardwareMap, 0x3a, telemetry);

        Thread.sleep(1000);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Play scenario 1
        playScenario1();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //      telemetry.addData("First", "First sec");
        //       encoderDrive(DRIVE_SPEED, 15,  15);  // S1: Forward 47 Inches with 5 Sec timeout
        //       turn(TURN_SPEED, 0, 24);  // S1: Forward 47 Inches with 5 Sec timeout
        //       readColorSensor();
        //       telemetry.update();

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
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        float hsvValues[] = {0, 0, 0};
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            sensor_colorA.enableLed();
            sensor_colorB.enableLed();

            int counter = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    robot.leftMotor.getCurrentPosition() < newLeftTarget &&
                    robot.rightMotor.getCurrentPosition() < newRightTarget) {

                sensor_colorA.printValues();
                sensor_colorB.printValues();

                if (sensor_colorA.isWhite()) {
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    telemetry.addData("ALB: ", "DETECTED!!!");
                    telemetry.addData("Counter: ", counter);
                    //End color sensor
                    telemetry.update();
                    sleep(2050);
                    break;
                }
                counter++;
                telemetry.addData("Counter: ", counter);
                //End color sensor
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void colectare(double speed) {

        while (opModeIsActive() && runtime.seconds() < 1) {

            robot.collectMotor1.setPower(-speed);
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());

        }
        robot.collectMotor1.setPower(0);
        runtime.reset();
    }


    public void goTowardsBeacon(double speed,
                                double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        float hsvValues[] = {0, 0, 0};
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            sensor_colorA.enableLed();
            sensor_colorB.enableLed();

            int counter = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive()) {

                sensor_colorA.printValues();
                sensor_colorB.printValues();
                if (sensor_colorB.isRed()) {
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    telemetry.addData("RED: ", "DETECTED!!!");
                    telemetry.addData("Counter: ", counter);
                    //End color sensor
                    telemetry.update();
                    sleep(2050);
                    break;
                } else if (sensor_colorB.isBlue()) {
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    telemetry.addData("RED: ", "DETECTED!!!");
                    telemetry.addData("Counter: ", counter);
                    //End color sensor
                    telemetry.update();
                    sleep(2050);
                    break;
                }
                counter++;
                telemetry.addData("Counter: ", counter);
                //End color sensor
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void turn(double speed,
                     double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        float hsvValuesA[] = {0F, 0F, 0F};
        float hsvValuesB[] = {0F, 0F, 0F};

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + 10000;//(int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + 10000;//(int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
          // robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(0);

            int counter = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            //while (opModeIsActive() && robot.leftMotor.getCurrentPosition() < newLeftTarget || robot.rightMotor.getCurrentPosition() < -newRightTarget)
            while (opModeIsActive() && (robot.leftMotor.isBusy() || robot.rightMotor.isBusy()) && robot.leftMotor.getCurrentPosition()<leftInches) {


                    if(robot.leftMotor.getCurrentPosition()>=leftInches){
                        telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                        telemetry.addData("Right motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                robot.leftMotor.getCurrentPosition(),
                                robot.rightMotor.getCurrentPosition());
                        break;

                    }
                telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                telemetry.addData("Right motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());

                //    telemetry.addData("Hue", hsvValuesB[0]);
                counter++;
                telemetry.addData("Counter: ", counter);
                //End color sensor
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void drive (double speed,
                     double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        float hsvValuesA[] = {0F, 0F, 0F};
        float hsvValuesB[] = {0F, 0F, 0F};

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + 10000;//(int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + 10000;//(int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            int counter = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            //while (opModeIsActive() && robot.leftMotor.getCurrentPosition() < newLeftTarget || robot.rightMotor.getCurrentPosition() < -newRightTarget)
            while (opModeIsActive() && (robot.leftMotor.isBusy() || robot.rightMotor.isBusy()) && (robot.leftMotor.getCurrentPosition()<leftInches &&
                    robot.rightMotor.getCurrentPosition()<rightInches)) {


                if(robot.leftMotor.getCurrentPosition()>=leftInches && robot.rightMotor.getCurrentPosition()>=rightInches){
                    telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                    telemetry.addData("Right motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    break;

                }
                telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                telemetry.addData("Right motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());

                //    telemetry.addData("Hue", hsvValuesB[0]);
                counter++;
                telemetry.addData("Counter: ", counter);
                //End color sensor
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }



    public void trage(double speed, double leftInches) {

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
            while (opModeIsActive() && robot.armMotor.isBusy()) {
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

    public void encoderMove(double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            if(((newLeftTarget-robot.leftMotor.getCurrentPosition()<=30 || robot.leftMotor.getCurrentPosition()-newLeftTarget  <=30))||
                    (robot.rightMotor.getCurrentPosition()-newRightTarget>=30 ||
                            newRightTarget-robot.rightMotor.getCurrentPosition() >=30)) {


                robot.leftMotor.setTargetPosition(Math.abs(newLeftTarget- robot.leftMotor.getCurrentPosition()));
                robot.rightMotor.setTargetPosition(Math.abs(newRightTarget-robot.rightMotor.getCurrentPosition()));

                // Turn On RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                robot.leftMotor.setPower(Math.abs(speed));
                robot.rightMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                while ((opModeIsActive()) && ((robot.leftMotor.isBusy() || robot.rightMotor.isBusy()))) {


                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(5000);   // optional pause after each move
            }else {

                robot.leftMotor.setTargetPosition(newLeftTarget);
                robot.rightMotor.setTargetPosition(newRightTarget);

             /*   // Turn On RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

                // reset the timeout time and start motion.
                robot.leftMotor.setPower(Math.abs(speed));
                robot.rightMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                while ((opModeIsActive()) && ((robot.leftMotor.isBusy() || robot.rightMotor.isBusy()))) {


                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(5000);   // optional pause after each move
            }
        }
    }


    private void readColorSensor() {
        int counter = 0;
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        sensor_colorA.enableLed();
        sensor_colorB.enableLed();
        while (counter < 5) {

            sensor_colorA.printValues();
            //  sensor_colorB.printValues();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            telemetry.addData("Counter  ", counter++);
            telemetry.update();
            //counter++;
        }
    }

    public void encoderDriveWithoutSensor (double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        float hsvValues[] = {0, 0, 0};
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches );
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches );
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));


            int counter = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    Math.abs(newLeftTarget - robot.leftMotor.getCurrentPosition()) < 10 &&
                    Math.abs(newRightTarget - robot.rightMotor.getCurrentPosition()) < 10) {
                counter++;
                telemetry.addData("Counter: ", counter);
                //End color sensor
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //daca nu sunt erori va zice ca e bine (true)
    private boolean playScenario1() {
        sleep(1000);
        encoderDriveWithoutSensor(DRIVE_SPEED,1120,1120 );//viteza ticksuri roata stanga , ticksuri roata dreapta

      /*  sleep(500);
        telemetry.addData("faza", "mers pana la aruncare bile");
        telemetry.update();
        drive(DRIVE_SPEED / 2,180 , 180);
        telemetry.update();


        sleep(500);
        telemetry.addData("faza", "intoarcere");
        telemetry.update();
        turn(DRIVE_SPEED / 3,560 , 0);
        telemetry.update();

        sleep(500);
        telemetry.addData("faza", "mers catre beacon");
        telemetry.update();
        drive(DRIVE_SPEED / 2,1120 , 1120);
        telemetry.update();*/


        /* // mergem pana la linia alba
        sleep(1000);
        telemetry.addData("faza", "mers pana la lina alba");
        telemetry.update();
        encoderDrive(DRIVE_SPEED / 2, 3100 / 2.54, 3100 / 2.54);
        telemetry.update();


        sleep(100);
        telemetry.addData("faza", "mers inainte pentru a catapulta");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 20, 20);
        telemetry.update();

        sleep(100);
        telemetry.addData("faza", "tragere si revenire catapulta o data");
        telemetry.update();
        trage(1, (-2700));
        telemetry.update();

        sleep(100);
        runtime.reset();
        telemetry.addData("faza", "colectare");
        telemetry.update();
        colectare(1);
        telemetry.update();

        sleep(500);
        trage(1, (-2700));

        //ne intoarcem la 45 grade pe loc
        sleep(1000);
        telemetry.addData("faza", "intoarcere 45");
        telemetry.update();
        turn(TURN_SPEED / 3, 24, -24);
        telemetry.update();


        //ne intoarcem 90grade
        sleep(1000);
        telemetry.addData("faza", "intoarcere");
        telemetry.update();
        turn(TURN_SPEED / 3, 24, -24); // deocamdata nu face ce trebe
        //sleep(2050);
        // merge in fata pana la beacon - acum 10 inch
        // mergem pana zice stop senzorul de atingere

        sleep(1000);
        telemetry.addData("faza", "mers pana la beacon");
        telemetry.update();
        goTowardsBeacon(DRIVE_SPEED, 3100 / 2.54, 3100 / 2.54);
        telemetry.update();*/



        /*//citim culoare din senzor B
        telemetry.addData("faza", "citire culoare");
        telemetry.update();


        readColorSensor();
        telemetry.addData("Sfarsit", "da");
        telemetry.update();
*/
        return true;
    }


}
