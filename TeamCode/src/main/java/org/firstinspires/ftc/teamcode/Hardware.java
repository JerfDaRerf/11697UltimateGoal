package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 ***  Hardware Motor Controllers Settings ***
 RL Controller (AL025T7Z)
 0 - NeveRest 40 Gearmotor (FLMotor)
 RR Controller (AL025T80)
 1 - NeveRest 40 Gearmotor (FRMotor)
 FL Controller (A7008KTV)
 2 - NeveRest 40 Gearmotor (LRMotor)
 FR Controller (A7008KBB)
 3 - NeveRest 40 Gearmotor (RRMotor)
 */

public class Hardware

        /**Unnecessary values are left as examples -- DON'T DELETE */

{
    //Define values for servos
    public static final double stabbyIniOpen = 0.00;


    /* Public Motors */
    public DcMotor frontLeftMotor      = null;     // H2 channel 0     FLMotor
    public DcMotor frontRightMotor     = null;     // H2 channel 1     FRMotor
    public DcMotor rearLeftMotor       = null;     // H2 channel 2     RLMotor
    public DcMotor rearRightMotor      = null;     // H2 channel 3     RRMotor
    public DcMotor SuckyBoi            = null;     // H3 channel 0     SuckyBoi
    public DcMotor DonutBoi            = null;     // H3 channel 1     DonutBoi
    public DcMotor ShootyBoi           = null;     // H3 channel 2     ShootyBoi
    public DcMotor TapePark           = null;     // H3 channel 3     ResidentSleeper

    public CRServo IntakePush          = null; //H2 channel 0 IntakePush
    public Servo RingPusher            = null; //H2 channel 1 RingPusher
    public Servo BajelBoi              = null; //H2 channel 2 Bajel
    public Servo BajuetteBoi           = null; //H2 channel 3 BajuetteBoi
    public CRServo StuckBoi            = null; //H2 channel 4 GetGood
    public Servo IntakeControl                = null; // H2 Channel 5 Dubois

    public TouchSensor cantTouchThis   = null;    // H2 port 0        cantTouchThis

    BNO055IMU imu;


    /* local OpMode members. */
    HardwareMap hwMap             =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("LF_Motor");
        frontRightMotor = hwMap.dcMotor.get("RF_Motor");
        rearLeftMotor   = hwMap.dcMotor.get("LR_Motor");
        rearRightMotor  = hwMap.dcMotor.get("RR_Motor");
        SuckyBoi        = hwMap.dcMotor.get("SuckyBoi");
        DonutBoi        = hwMap.dcMotor.get("DonutBoi");
        ShootyBoi         = hwMap.dcMotor.get("ShootyBoi");
        TapePark         = hwMap.dcMotor.get("ResidentSleeper");


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SuckyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DonutBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShootyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TapePark.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SuckyBoi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DonutBoi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShootyBoi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TapePark.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        SuckyBoi.setPower(0);
        DonutBoi.setPower(0);
        ShootyBoi.setPower(0);
        TapePark.setPower(0);

        // IntakePush.setPower(0);

        //Servo config
        RingPusher         = hwMap.get(Servo.class, "RingPusher");
        BajelBoi         = hwMap.get(Servo.class, "Bajel");
        BajuetteBoi         = hwMap.get(Servo.class, "Bajuette");
        IntakePush         = hwMap.get(CRServo.class, "IntakePush");
        StuckBoi         = hwMap.get(CRServo.class, "GetGood");
        IntakeControl         = hwMap.get(Servo.class, "Dubois");



        //SENSOR CONFIG
        cantTouchThis = hwMap.get(TouchSensor.class,"cantTouchThis");


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