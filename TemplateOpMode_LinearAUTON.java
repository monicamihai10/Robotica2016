package org.firstinspires.ftc.robotcontroller.external.samples;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;




/*
 * Created by ADMIN on 2/11/2017.
 */



@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class TemplateOpMode_LinearAUTON extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();

        DcMotor motorCTP = null;
        DcMotor leftMotor = null;
        DcMotor rightMotor = null;
        TouchSensor ts;
        ModernRoboticsI2cRangeSensor rs;

        rs = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        ts = hardwareMap.touchSensor.get("sensor_touch");
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        motorCTP = hardwareMap.dcMotor.get("left_arm");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //initial erau invers. vezi pana a urma care-i fata

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCTP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCTP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        motorCTP.setPower(0);




        int state= 1;
        int distUS = 255, distOPT = -1;
        boolean touch = false;

        waitForStart();
        runtime.reset();


        //AICI LUCRAM NOI
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())

        {

            switch (state)
            {
                case 1:
                    sleep(200);
                    leftMotor.setPower(0.5);
                    rightMotor.setPower(0.5);
                    sleep(500);
                    telemetry.addData("state" , "is:" + state);
                    telemetry.addData("ticks", "rightmotor:" + rightMotor.getCurrentPosition());
                    telemetry.addData("button", "pressed:" + ts.isPressed());
                    telemetry.addData("touch", "pressed:" + touch);
                    telemetry.update();
                    if(rightMotor.getCurrentPosition()>=100)
                    {
                        rightMotor.setPower(0);
                        leftMotor.setPower(0);
                    }
                    sleep(5000);
                    //ne imaginam ca arunca cu catapulta
                    state++;

                    break;

                case 2:
                    telemetry.addData("state" , "is:" + state);
                    telemetry.addData("ticks", "rightmotor:" + rightMotor.getCurrentPosition());
                    telemetry.addData("button", "pressed:" + ts.isPressed());
                    telemetry.addData("touch", "pressed:" + touch);
                    telemetry.update();
                    if(ts.isPressed()) touch = true;
                    leftMotor.setPower(1);
                    rightMotor.setPower(1);
                    if(touch)
                    {
                        rightMotor.setPower(0);
                        leftMotor.setPower(0);
                    }
                    state++;

                    break;

            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());








        }
    }
}