package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by monica on 09.03.2017.
 */

public class HardwareAutonomus {

    public DcMotor leftMotor   = null;
    public DcMotor  rightMotor  = null;
    //motor catapulta
    public DcMotor  armMotor    = null;
    //motor lift
    public DcMotor liftMotor = null;

    //2 motoare pt maturica
    public DcMotor collectMotor1 = null;


    public final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.75 ;//AM MOD DE LA 0.45 CU +
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // era 720 valoarea e 140 PPR la Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.4;
    public static final double     TURN_SPEED              = 0.4;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareAutonomus(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        armMotor    = hwMap.dcMotor.get("left_arm");

        liftMotor =hwMap.dcMotor.get("lift_ball");

        collectMotor1 =hwMap.dcMotor.get("up_collect");
        //collectMotor2 =hwMap.dcMotor.get("down_collect");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        liftMotor.setDirection(DcMotor.Direction.FORWARD);//nu stim directia =>de schimbat daca merge pe dos

        collectMotor1.setDirection(DcMotor.Direction.FORWARD);
        // collectMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        armMotor.setPower(0);

        liftMotor.setPower(0);

        collectMotor1.setPower(0);
        //collectMotor2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
