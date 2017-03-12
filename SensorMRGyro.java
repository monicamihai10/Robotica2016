package org.firstinspires.ftc.robotcontroller.external.samples;

/*
Modern Robotics Gyro Turn Example
Updated 11/3/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Gyro Sensor named "gyro"
Motor named "ml" for left drive train
Motor named "mr" for right drive train

For more information, go to http://modernroboticsedu.com/course/view.php?id=4
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//@Disabled
public class SensorMRGyro extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();


    int zAccumulated; //Total rotation left/right
    int heading;
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro gyro;

    public SensorMRGyro() {
        this.sensorGyro = sensorGyro;
    }




    @Override
    public void runOpMode() {
        // robot.rightMotor.setDirection(DcMotor.Direction.REVERSE); //decomendeaza daca nu merg bine motoarele

        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorGyro = hardwareMap.gyroSensor.get("gyro_sensor");
        gyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        gyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        waitForStart();
        runtime.reset();

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        while (gyro.isCalibrating()) {}
            //Ensure calibration is complete (usually 2 seconds)


        while(opModeIsActive()){
            turnAbsolute(45);
            sleep(1000);
            try {
                turn(-45);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + gyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {

        zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
        heading = gyro.getHeading();
        double turnSpeed = 0.1;

        while (Math.abs(zAccumulated - target) > 5 && opModeIsActive()) {//Continue while the robot direction is further than three degrees from the target

            telemetry.addData("sunt in while ar trebui sa ma misc"," adevarat");

            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                robot.leftMotor.setPower(-turnSpeed);
                robot.rightMotor.setPower(turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                robot.leftMotor.setPower(turnSpeed);
                robot.rightMotor.setPower(-turnSpeed);
            }

            zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
            heading = gyro.getHeading();  //Set variables to gyro readings

            telemetry.addData("accumulated", String.format("%03d", zAccumulated));
            telemetry.addData("heading", String.format("%03d", heading));
            telemetry.update();
        }

        robot.rightMotor.setPower(0);  //Stop the motors
        robot.leftMotor.setPower(0);

    }


    public void init(HardwareMap hardwareMap) {
    }
}