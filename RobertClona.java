package org.firstinspires.ftc.robotcontroller.external.samples;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="RobertClona", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class RobertClona extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        //AICI INITIALIZAM



        //DcMotor motor_class = null;
        //DcMotor motorFS = null;
        // DcMotor motorFD = null;
        int loc_aruncare = 1220;
        //VERIFICA CAT TREBIE SA FIE LOC ARUNCARE

        //GyroSensor giroscop = null;
        //giroscop = hardwareMap.gyroSensor.get("giroscop");

        // giroscop.calibrate();


        //TouchSensor atingere_fatza = null;
        //atingere_fatza = hardwareMap.touchSensor.get("atingere");



        DcMotor motorSS = null;
        DcMotor motorSD = null;
        DcMotor motorCTP = null;
        //VREI ENCODER PT CATAPULTA PT CA VREI SA VERIFICI CAND E ARMATA CATAPULTA
        //motorFS = hardwareMap.dcMotor.get("motorFS");
        //motorFD = hardwareMap.dcMotor.get("motorFD");
        motorSS = hardwareMap.dcMotor.get("left_drive");
        //SD= SPATE DREAPTA, SS= SPATE STANGA, FS= FATA STANGA, FD= FATA DREAPTA
        motorSD = hardwareMap.dcMotor.get("right_drive");
        motorCTP = hardwareMap.dcMotor.get("left_arm");
        int ticksSS=0,ticksSD=0,ticksCTP=0;
        int pos = 0;
        final double minpos=0.0, maxpos=1.0, increment=0.1;

        int rotSS=0,rotSD=0;
        final int tetrix=1120, ondrasmarc=1440;
        motorSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motorCTP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //MAI BINE IL PUI LA FIECARE RULARE A CODULUI IN POZITIA 0 SI DACA NU MERGE, ATUNCI FOLOSIM SISTEMUL DE TICKURI
        motorCTP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCTP.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorCTP.setTargetPosition(0);

        //Servo ridicare1 = hardwareMap.servo.get("ridicare1");
       //Servo ridicare2 = hardwareMap.servo.get("ridicare2");

        //ridicare1.setPosition(minpos);
        //ridicare2.setPosition(minpos);

       // double rid1pos = ridicare1.getPosition(), rid2pos = ridicare2.getPosition();
        boolean directie1=true,directie2=true;

        waitForStart();
        runtime.reset();



        while(opModeIsActive())

        {
            // MISCAREA LA STANGA dupa o sg roata

            if(gamepad1.left_stick_x < 0 && gamepad1.right_stick_x == 0)
            {
                motorSS.setPower(gamepad1.left_stick_x);
            }
            else
            {
                //misca motorul stang in spate
                motorSS.setPower(0);
                motorSD.setPower(0);
            }



            if(gamepad1.left_stick_x == 0 && gamepad1.right_stick_x < 0)
            {
                //motor drept in fatza
                motorSD.setPower(-gamepad1.right_stick_x);
            }
            else
            {
                motorSS.setPower(0);
                motorSD.setPower(0);
            }

            //misca-l la DREAPTA dupa o sg roata

            if(gamepad1.left_stick_x > 0 && gamepad1.right_stick_x == 0)
            {
                //motor stang fatza
                motorSS.setPower(gamepad1.left_stick_x);
            }
            else
            {

                motorSS.setPower(0);
                motorSD.setPower(0);
            }

            //misca-l la DREAPTA dupa motorul drept

            if(gamepad1.left_stick_x == 0 && gamepad1.right_stick_x > 0)
            {
                //motor drept in spate
                motorSD.setPower(-gamepad1.right_stick_x);
            }
            else
            {
                motorSS.setPower(0);
                motorSD.setPower(0);
            }

            //rotire pe loc

            if( ( gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0 ) || (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0) )
            {
                motorSD.setPower(-gamepad1.left_stick_x);
                motorSS.setPower(-gamepad1.left_stick_x);

            }
            else
            {
                motorSS.setPower(0);
                motorSD.setPower(0);

            }

            //miscare in fatza

            if ( gamepad1.left_stick_y != 0)
            {
                motorSD.setPower(-gamepad1.left_stick_y);
                motorSS.setPower(gamepad1.left_stick_y);
            }
            if ( gamepad1.right_stick_y != 0)
            {
                motorSD.setPower(-gamepad1.right_stick_y);
                motorSS.setPower(gamepad1.right_stick_y);
            }

            //CODU DE CATAPULTA:

            pos = motorCTP.getCurrentPosition();

            if(gamepad1.a && ( !motorCTP.isBusy() ) )
            {
                motorCTP.setTargetPosition(pos + ondrasmarc);
            }

        }



        //tre' sa vezi in ce pozitie e catapulta ca sa nu resetezi encoderul pe 0, in alta pozitie decat ar fi encoderul
        //zero..

        // run until the end of the match (driver presses STOP)

    }
}
