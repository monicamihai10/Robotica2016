package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Encoder extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot(); //am facut legatura cu robot.
    private ElapsedTime runtime = new ElapsedTime();     //am adaugat runtime
    static final double COUNTS_PER_MOTOR_REV = 560;   //am definit o rotatie completa a rotii
    int Tiks_per_roata = 560;                                 //am pus de tip int pentru ca functia set target are nevoie de int
    int start_position_right;
    int start_position_left;
    int eroare_ticks = 10;
    double speed = 0.3;


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    //resetez encoderele la pornirea programului
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(1000); // sleep ca sa se reseteze encoderele

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);           //setez modul pe run to position
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        start_position_left = robot.leftMotor.getCurrentPosition();         //imi iau referinta
        start_position_right = robot.rightMotor.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            /*robot.rightMotor.setTargetPosition(robot.rightMotor.getCurrentPosition() + Tiks_per_roata);
            robot.leftMotor.setTargetPosition(robot.leftMotor.getCurrentPosition() + Tiks_per_roata);

            robot.rightMotor.setPower(0.4);
            robot.leftMotor.setPower(0.4);

            while (Math.abs(Tiks_per_roata - robot.leftMotor.getCurrentPosition()) < eroare_ticks) {

                telemetry.addData("Rotile sunt ocupate, se duc catre pozitia dorita", "");
                telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
                telemetry.addData("Right motor: ", "Value:" + robot.rightMotor.getCurrentPosition());

            }

            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);*/


            deplasare_robot(speed,560,560,10);


        }


    }


    public void deplasare_robot(double speed, int LeftTicks, int RightTicks , int eroare_ticks) {


        //TELEMETRY

        telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
        telemetry.addData("Right motor: ", "Value:" + robot.rightMotor.getCurrentPosition());

        //TELEMETRY

        robot.rightMotor.setTargetPosition(robot.rightMotor.getCurrentPosition() + LeftTicks);
        robot.leftMotor.setTargetPosition(robot.leftMotor.getCurrentPosition() + RightTicks);

        robot.rightMotor.setPower(speed);
        robot.leftMotor.setPower(speed);

        while (Math.abs(Tiks_per_roata - robot.leftMotor.getCurrentPosition()) < eroare_ticks) {

            telemetry.addData("Rotile sunt ocupate, se duc catre pozitia dorita", "");
            telemetry.addData("Left motor: ", "Value:" + robot.leftMotor.getCurrentPosition());
            telemetry.addData("Right motor: ", "Value:" + robot.rightMotor.getCurrentPosition());


        }

        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);




    }



}