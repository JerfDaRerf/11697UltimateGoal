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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

@Autonomous(name="BlueRightRingSide", group ="11697")
//BLUE Left has "public class BlueLeft extends LinearOpMode
public class BlueRightRingSide extends LinearOpMode {
    Hardware robot = new Hardware();
    //SEASON EXCLUSIVE
    private boolean completedShot = false;
    RingHeight height;

    // Motors Settings [DO NOT CHANGE]
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Used by system functions
    private ElapsedTime moveTimer = new ElapsedTime();
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    private String PREVIOUS_MSG = "";
    private String numRings = "";

    /*
       Vuforia License Key can be found at:
       https://developer.vuforia.com/license-manager
     */

    /**
     * VALUES FOR RING DETECTION
     */
    private static final String VUFORIA_KEY = "AeWcyMH/////AAABmUkuasHKa0f1i89tz3zGcElfn6Z6Hp6yTh0hLzX27O+whXiOv14KF2u81LZhtuCIsiDoiNDM3YKIWl3wNl6dv5SuDu7iHPX1xeqaMRQasLapwfvkD3wLtzMGrh5f1Y1BhsZ/DfsVI0kaa9c0rdpSYH08nZaBvLWpnVLA4a/7aZs1jenfcakcuQa0rvF4ee8Y995jxkrOebg63rjNhDaWKvlu7LSwOBGl0/SXQufK199qioVPIB1eqAv6IYoMmro7xTjh3O6bey8aY21ZcyhKuz5J0TnIYZhbPhRfkgktrRspkOdGlLN5nLPwBJemoiKzASZblUIM6Evj6ICmIcxkny8dOlSMZeA6+OGVADKkYk85\n";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public enum RingHeight {
        NONE,
        ONE,
        FOUR
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        sleep(300);
        robot.BajelBoi.setPosition(0.725);
        robot.BajuetteBoi.setPosition(0.915);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        sleep(1000);
        ringDetect(tfod);
        sleep(1500);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // Calibrate IMU before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("DONEDONEDONEDONE", "DONEDONEDONEIT'SDONEE");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            ringDetect(tfod);
            //driveByEncoder(0.65, 6.96, 0);
            //driveByEncoder(1, 3.1, 0);
             //shoot power shot
            driveByEncoder(0.7, 28, 28);
            robot.ShootyBoi.setPower(-0.67);
            sleep(100);
            //rotate for right PS
            driveByEncoder(0.45, 0, 3.725);
            //sleep(100);
            shoot();

            driveByEncoder(0.3, -0.8,0.8);
            sleep(150);
            shoot();
            driveByEncoder(0.3, -0.7,0.7);
            sleep(150);
            shoot();
            driveByEncoder(0.3,1.5,-1.5);
//            driveByEncoder(0.45, 0, 3.5);
//            //sleep(100);
//            shoot();
//            //driveByEncoder(0.4, 0.2, -0.2);
//            driveByEncoder(0.4, 0, 4.1);
//            shoot();

            /*if (completedShot == true) {
                driveByEncoder(0.55, 3.675, 0);
                completedShot = false;
            }*/
            //shoot
            //shoot();


            robot.ShootyBoi.setPower(0);
            driveByEncoder(0.4, 7.675, 0);


            switch (numRings) {
                case "FOUR":
                //wobble
                    driveByEncoder(1, 5, 5);
                    driveByEncoder(0.8, 7.9,0);
                    //driveByEncoder(0.8, 15.5, 0);
                    sleep(500);
                    robot.RingPusher.setPosition(0.9);
                    robot.ShootyBoi.setPower(-0.75);
                    robot.DonutBoi.setPower(1);
                    robot.SuckyBoi.setPower(1);
                    robot.StuckBoi.setPower(-1);
                    sleep(1200);
                    driveByEncoder(0.95, -15, -15);
                    driveByEncoder(0.7, -3.75, -3.75);
                    driveByEncoder(0.95, 13, 13);
                    //sleep(500);
                    //turn intake off

                    robot.DonutBoi.setPower(0);
                    robot.SuckyBoi.setPower(0);
                    robot.StuckBoi.setPower(0);
                    driveByEncoder(0.65, 0, 6);

                    //shoot
                    for(int i = 0; i < 3; i++){
                        shoot();
                        if (completedShot == true) {

                            completedShot = false;
                        }
                    }
                robot.ShootyBoi.setPower(0);
                //driveByEncoder(0.8, 16, 0);
                driveByEncoder(0.65, 0.75, 0);
                driveByEncoder(0.4, 3, -3);
                driveByEncoder(1, 26, 26);
                robot.BajelBoi.setPosition(0.354);
                sleep(1500);
                robot.BajuetteBoi.setPosition(0.765);
                sleep(500);
                driveByEncoder(1, -18, -18);

                break;

                case "ONE":
                    telemetry.addData("SINGLESINGLE", 1);
                    telemetry.update();
                    driveByEncoder(1, 5, 5);
                    driveByEncoder(0.8,7.9,0);
                    //driveByEncoder(0.8, 15.5, 0);
                    robot.RingPusher.setPosition(0.9);
                    robot.ShootyBoi.setPower(-0.85);
                    robot.DonutBoi.setPower(1);
                    robot.SuckyBoi.setPower(1);
                    robot.StuckBoi.setPower(-1);
                    sleep(969);
                    driveByEncoder(0.95, -8.7, -8.7);
                    driveByEncoder(0.6, -4, -4);
                    driveByEncoder(0.95, 7, 7);
                    //sleep(500);
                    //turn intake off
                    robot.DonutBoi.setPower(0);
                    robot.SuckyBoi.setPower(0);
                    robot.StuckBoi.setPower(0);
                    driveByEncoder(0.65, 0, 6);
                    for(int i = 0; i < 1; i++){
                        shoot();
                        if (completedShot == true) {

                            completedShot = false;
                        }
                    }
                    robot.ShootyBoi.setPower(0);
                    driveByEncoder(0.8, 1, 0);
                    driveByEncoder(0.9, 11.5, 11.5);
                    robot.BajelBoi.setPosition(0.354);
                    sleep(1500);
                    robot.BajuetteBoi.setPosition(0.765);
                    sleep(200);
                    driveByEncoder(1, -6, -6);

                    break;

                case "ZERO":
                    telemetry.addData("00000", 1);
                    robot.ShootyBoi.setPower(0);
                    driveByEncoder(1, 12,12);
                    driveByEncoder(0.4, 13, -13);
                    driveByEncoder(1, 8.75, 8.75);
                    robot.BajelBoi.setPosition(0.354);
                    sleep(1500);
                    robot.BajuetteBoi.setPosition(0.765);
                    sleep(500);
                    driveByEncoder(1, -10, -100);
                    driveByEncoder(0.8, -13, 13);
                    driveByEncoder(1,-3,-3);

                    break;
                default:

                    break;
            }
//            robot.RingPusher.setPosition(0.1);
//            sleep(5000);
//            robot.TapePark.setPower(-1);
//            sleep(3500);
            break;


        }

    }




    /*
     **************************************************
     **************USER DEFINED FUNCTIONS:*************
     **************************************************
     */


    private void shoot(){

        robot.IntakePush.setPower(0);
        sleep(800);
        while (robot.cantTouchThis.isPressed() == false){
            robot.IntakePush.setPower(1);
        }
        if(robot.cantTouchThis.isPressed() == true) {
            sleep(450);
            robot.IntakePush.setPower(0);
            completedShot = true;
            return;
        }


    }

    private void driveByEncoder(double speed, double leftInches, double rightInches) throws InterruptedException {

        /**********************************************************
         driveByEncoder(0.3, 10.0, 10.0);            // Forward
         driveByEncoder(0.3, -10.0, -10.0);          // Backward
         driveByEncoder(0.3, 0, 10.0);               // Shift Right
         driveByEncoder(0.3, 10.0, 0);               // Shift Left
         driveByEncoder(0.3, 22.0, -22.0);           // Turn Right
         driveByEncoder(0.3, -22.0, 22.0);           // Turn Left
         ***********************************************************/
        speed /=2;
        leftInches /=2;
        rightInches /=2;
        String robotAction = "";
        int newLeftTarget;
        int newRightTarget;

        if (leftInches < 0 && rightInches < 0) {
            robotAction = "BACKWARD";
        } else if (leftInches > 0 && rightInches > 0) {
            robotAction = "FORWARD";
        } else if (leftInches > 0 && rightInches == 0) {
            robotAction = "SHIFT_LEFT";
        } else if (leftInches == 0 && rightInches > 0) {
            robotAction = "SHIFT_RIGHT";
        } else if (leftInches < 0 && rightInches > 0) {
            robotAction = "TURN_LEFT";
        } else if (leftInches > 0 && rightInches < 0) {
            robotAction = "TURN_RIGHT";
        } else {
            return;
        }

        // Remember current motors direction, will reset in the end
        DcMotor.Direction dirFL = robot.frontLeftMotor.getDirection();
        DcMotor.Direction dirFR = robot.frontRightMotor.getDirection();
        DcMotor.Direction dirRL = robot.rearLeftMotor.getDirection();
        DcMotor.Direction dirRR = robot.rearRightMotor.getDirection();
        DcMotor.RunMode runModeFL = robot.frontLeftMotor.getMode();
        DcMotor.RunMode runModeFR = robot.frontRightMotor.getMode();
        DcMotor.RunMode runModeRL = robot.rearLeftMotor.getMode();
        DcMotor.RunMode runModeRR = robot.rearRightMotor.getMode();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // power is removed from the motor, set the current encoder position to zero
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All mortors will move forward
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
        //logMessage("curFL,curFR",  robot.frontLeftMotor.getCurrentPosition() +", "+ robot.frontRightMotor.getCurrentPosition());

        switch (robotAction) {

            case "FORWARD":
                //logMessage("Moving Robot", "FORWARD");
                break;

            case "BACKWARD": // mortor direction aame as FORWAED, because encoder will be "-"
                //logMessage("Moving Robot", "BACKWARD");
                break;

            case "SHIFT_LEFT":
                //logMessage("Moving Robot", "SHIFT_LEFT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //+
                robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //+
                robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //+
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //+
                //robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                newRightTarget = newLeftTarget;
                break;

            case "SHIFT_RIGHT":
                //logMessage("Moving Robot", "SHIFT_RIGHT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //-
                robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //-
                robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //-
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //-
                //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                newLeftTarget = newRightTarget;
                break;

            case "TURN_LEFT":
                //logMessage("Moving Robot", "TURN_LEFT");
                break;

            case "TURN_RIGHT":
                //logMessage("Moving Robot", "TURN_RIGHT");
                break;

        }

        robot.frontLeftMotor.setTargetPosition(newLeftTarget);
        robot.frontRightMotor.setTargetPosition(newRightTarget);
        robot.rearLeftMotor.setTargetPosition(newLeftTarget);
        robot.rearRightMotor.setTargetPosition(newRightTarget);
        //logMessage("newLeftTarget,newRightTarget",  newLeftTarget +", "+ newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the motor speed and start motion
        robot.frontLeftMotor.setPower(Math.abs(speed));
        robot.frontRightMotor.setPower(Math.abs(speed));
        robot.rearLeftMotor.setPower(Math.abs(speed));
        robot.rearRightMotor.setPower(Math.abs(speed));

        //Ramp up motor speed to match target
//        while(power <= speed) {
//            power += RAMP_INCREMENT;
//        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) {

            /*
            logMessage("Path1",  newLeftTarget +", "+ newRightTarget);
            logMessage("Path2",
                    robot.frontLeftMotor.getCurrentPosition() + ", " +
                    robot.frontRightMotor.getCurrentPosition() + ", " +
                            robot.rearLeftMotor.getCurrentPosition() + ", " +
                            robot.rearRightMotor.getCurrentPosition());
            */
        }


        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset back motors direction
        robot.frontLeftMotor.setDirection(dirFL);
        robot.frontRightMotor.setDirection(dirFR);
        robot.rearLeftMotor.setDirection(dirRL);
        robot.rearRightMotor.setDirection(dirRR);
        robot.frontLeftMotor.setMode(runModeFL);
        robot.frontRightMotor.setMode(runModeFR);
        robot.rearLeftMotor.setMode(runModeRL);
        robot.rearRightMotor.setMode(runModeRR);

    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    //Movement path for rightmost two blocks
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

//ORIGINAL CONFIDENCE 0.8
        tfodParameters.minResultConfidence = 0.45f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /** Do ring detection thingy */
    private void ringDetect(TFObjectDetector tfod) {
        numRings = "";
        telemetry.addData("ringDetect works", 1);
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            telemetry.addData("TFOD NOT NULL", 1);

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            telemetry.addData("# Object Detected", updatedRecognitions.size());

            if (updatedRecognitions != null) {
                if(updatedRecognitions.size() ==0 ){
                    height = RingHeight.NONE;
                    numRings = "ZERO";
                    return;
                }
                for (Recognition recognition : updatedRecognitions) {
                    //Recognition recognition = updatedRecognitions.get(0);
                    telemetry.addData("recognition:", recognition.getLabel());

                    if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                        height = RingHeight.FOUR;
                        numRings = "FOUR";
                        telemetry.addData("QUADQUAD", 1);

                    } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        height = RingHeight.ONE;
                        numRings = "ONE";
                        telemetry.addData("SINGLESINGLE", 1);

                    }

                    telemetry.update();

                }

            }
        }

    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle () {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle () {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate ( int degrees, double power){
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate
        // left power to left side
        robot.frontLeftMotor.setPower(leftPower);
        robot.rearLeftMotor.setPower(leftPower);

        robot.frontRightMotor.setPower(rightPower);
        robot.rearRightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


}