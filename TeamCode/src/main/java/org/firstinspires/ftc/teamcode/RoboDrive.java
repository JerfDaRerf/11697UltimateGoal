/*
/*
https://github.com/Rambotics/FTC-2016-2017-v2.4-pc/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode
https://github.com/pmtischler/ftc_app/tree/master/SharedCode/src/main/java/com/github/pmtischler
Code Folding Expand/Collapse ALL => Control || Shift || +/-
Last Edit Location => Control + Shift + BackSpace
Add Bookmark => Control + F11 + number
Find Bookmark => Control + number
Show Bookmarks => Shift + F11
Jump to Declaration => Control + B
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

//===========================================
// ***** Section 1          *****
// ***** Change OpMode Name *****
//===========================================

@TeleOp(name="RoboDrive", group="11697")
// @Disabled

public class RoboDrive extends LinearOpMode {

//===========================================
// ***** Section 2                *****
// ***** Declare global variables *****
//===========================================

    Hardware robot = new Hardware();

    // Motors Settings [DO NOT CHANGE]
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference

    float SPEED_RATE = 1.0f;       // Driving speed rate
    int curPosition;
    double curPosition1 = 0.0;
    boolean isPressed = false;
    int testprint = 1;
    boolean intakeMode = false;

    /* Declare Global Variables. */
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");

    /* Insert more stuff as needed
     *  Look at https://tinyurl.com/yb2xco82 for examples */


    /***** Main Code *****/

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("OMEGALUL", testprint);
        telemetry.update();
        robot.init(hardwareMap);

        /*
        robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        */
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        SPEED_RATE = 1.0f;

        logMessage("Status", "Initialized v4.0a - " + myDateFormat.format(today));


        waitForStart(); // Wait for the game to start (driver presses PLAY)

        String PREVIOUS_MSG = "";        // MesLog
        float lxValue = 0, lyValue = 0, rxValue = 0, ryValue = 0;
        //double          curPosition;

        while (opModeIsActive()) {

            // Drive the robot
            lxValue = getJoystickValue(gamepad1.left_stick_x);
            lyValue = getJoystickValue(-gamepad1.left_stick_y);
            rxValue = getJoystickValue(gamepad1.right_stick_x);
            ryValue = getJoystickValue(gamepad1.right_stick_y);
            driveByJoystick(-lxValue, lyValue, rxValue);

            //robot.pinchyBoi.setPosition(0);

            /********************************************
             *************GAMEPAD 1**********************
             ********************************************/
            if (gamepad1.a) {
                if(intakeMode == false){
                    intakeMode = true;
                } else {
                    intakeMode = false;
                }
            }
            if(intakeMode = true){
                robot.SuckyBoi.setPower(1);
                robot.DonutBoi.setPower(1);
            } else if(intakeMode = false){
                robot.SuckyBoi.setPower(0);
                robot.DonutBoi.setPower(0);
            }

        }
    }





//===========================================
// ***** Section 5              *****
// ***** User Defined Functions *****
//===========================================

    private void driveByJoystick ( double lxValue, double lyValue, double rxValue){
        double vD = Math.sqrt(Math.pow(lxValue, 2) + Math.pow(lyValue, 2));
        //double thetaD = Math.atan2(lyValue, lxValue);
        double thetaD = Math.atan2(lxValue, -lyValue);
        double vTheta = rxValue;

        // Convert desired motion to wheel power, with power clamping.
        Mecanum.Wheels wheels = Mecanum.motionToWheels(vD, thetaD, vTheta);
        robot.frontLeftMotor.setPower(wheels.frontLeft);
        robot.frontRightMotor.setPower(wheels.frontRight);
        robot.rearLeftMotor.setPower(wheels.backLeft);
        robot.rearRightMotor.setPower(wheels.backRight);

        //telemetry.addData("Status", "==> " + vD +", "+thetaD+", "+vTheta);
        //telemetry.update();
    }

    private void stopRobot () {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    private void logMessage (String myDescription, String myMessage) throws InterruptedException {

        telemetry.addData(myDescription, myMessage);
        telemetry.update();
        RobotLog.d("11697CW - " + myDescription + " : " + myMessage);

    }

    private float getJoystickValue ( float rawValue){
        //clip the power values so that it only goes from -1 to 1
        rawValue = Range.clip(rawValue, -1, 1);

        if (Math.abs(rawValue) < 0.1) {
            return 0;
        } else {
            return rawValue * SPEED_RATE;
        }
    }
/*
    private void seeingStone(){
        horizontalMotor.setTargetPosition(350);
        pinchyBoi.setPosition(0);
        verticalMotor.setTargetPosition(50); // raise 2 inches???
    }
 */

/*
//===========================================
// ***** Section 6              *****
// ***** Backup Un-used Code    *****
//===========================================
/********************************************
 if (gamepad1.dpad_up) {
 V_POSITION += INCREMENT ;
 if (V_POSITION >= robot.VArmHigh ) {
 V_POSITION = robot.VArmHigh;
 }
 robot.vArm.setPosition(V_POSITION);
 telemetry.addData("UP >> Servo Position", "%5.2f", V_POSITION);
 telemetry.update();
 //sleep(CYCLE_MS);
 robot.waitForTick(CYCLE_MS);
 }
 if (gamepad1.dpad_down) {
 V_POSITION -= INCREMENT ;
 if (V_POSITION <= robot.VArmLow ) {
 V_POSITION = robot.VArmLow;
 }
 robot.vArm.setPosition(V_POSITION);
 telemetry.addData("DOWN >> Servo Position", "%5.2f", V_POSITION);
 telemetry.update();
 //sleep(CYCLE_MS);
 robot.waitForTick(CYCLE_MS);
 }
 ********************************************/


}


