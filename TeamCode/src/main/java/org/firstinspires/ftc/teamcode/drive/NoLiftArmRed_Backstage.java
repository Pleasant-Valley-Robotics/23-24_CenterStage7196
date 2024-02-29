/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.abs;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.VisionCamera;
import org.firstinspires.ftc.teamcode.utility.CubeSide;
import org.firstinspires.ftc.teamcode.utility.FieldSide;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Gyro Auto Red Backstage No Lift", group="Robot")
//@Disabled
public class NoLiftArmRed_Backstage extends LinearOpMode {
    /* Declare OpMode members. */
    ColorSensor colorSensor1 = null;    // Hardware Device Object
    ColorSensor colorSensor2 = null;

    DcMotor FLDrive = null; // Front Left Drive Motor
    DcMotor FRDrive = null; // Front Right Drive Motor
    DcMotor BLDrive = null; // Back Left Drive Motor
    DcMotor BRDrive = null; // Back Right Drive Motor
    IMU imu = null; // Inertial Measurement Unit      // Control/Expansion Hub IMU

    CRServo spinny = null;
    CRServo upperDrop = null;
    CRServo lowerDrop = null;
    DcMotor liftDriveLeft = null;
    DcMotor liftDriveRight = null;

    DcMotor spinTake = null;

    private double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;

    private int     frontLeftTarget   = 0;

    private int     frontRightTarget   = 0;

    private int     backLeftTarget   = 0;

    private int     backRightTarget   = 0;

    private int     liftDriveLeftTarget = 0;

    private int liftDriveRightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    private ElapsedTime runtime = new ElapsedTime(); // Timer for tracking time

    // Constants for calculating encoder counts and speed

    //38.25'' length of slide from base of motor to its position when completely out.
    // 1.570" is diameter
    // that times pie = encoder counts.
    //26.9:1

    static final double LIFT_COUNTS_PER_MOTOR_REV = 28; // Encoder counts per motor revolution

    static final double LIFT_DRIVE_GEAR_REDUCTION = 26.9; // Gear ratio of 26.9:1 on the motor, 1:1 external gearing

    static final double LIFT_PINION_DIAMETER_INCHES = 1.57; // Diameter of the pinion driving the viper slide belt

    //lift encoders per inch. Multiple how far you want go by this value to move lift a certain amount.
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) / (LIFT_PINION_DIAMETER_INCHES * Math.PI);


    static final double COUNTS_PER_MOTOR_REV = 28; // Encoder counts per motor revolution
    static final double DRIVE_GEAR_REDUCTION = (((1+(46/17))) * (1+(46/11))); //Gear ratio of 19:1 gearbox.
    static final double WHEEL_DIAMETER_INCHES = 96/25.4; // Diameter of the robot's wheels in mm/(mm/in)
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.3;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.3;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        liftDriveLeft = hardwareMap.get(DcMotor.class, "liftDriveLeft");
        liftDriveRight = hardwareMap.get(DcMotor.class, "liftDriveRight");
        spinTake = hardwareMap.get(DcMotor.class, "spintake");

        //Initialize the intake system variables.
        spinny = hardwareMap.get(CRServo.class, "spinny");
        upperDrop = hardwareMap.get(CRServo.class, "upperDrop");
        lowerDrop = hardwareMap.get(CRServo.class, "lowerDrop");


        //Grab and store the reference for the camera.
        VisionCamera camera = new VisionCamera(hardwareMap, FieldSide.RedClose);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //Was FORWARD.
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        //Was REVERSE.
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        liftDriveLeft.setDirection(DcMotor.Direction.REVERSE);
        liftDriveRight.setDirection(DcMotor.Direction.REVERSE);
        spinTake.setDirection(DcMotor.Direction.FORWARD);
        spinny.setDirection(DcMotorSimple.Direction.FORWARD);
        upperDrop.setDirection(CRServo.Direction.FORWARD);
        lowerDrop.setDirection(CRServo.Direction.FORWARD);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinTake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftDriveLeft.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        liftDriveRight.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            //telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            //telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        imu.resetYaw();

        //Start the pipeline for detection.
        camera.enableCubePipeline();

        waitForStart();

        //Set up a variable to store the cubeside detected.
        //window size is the amount of times you want it to find a value in a row.
        CubeSide cubeSide = camera.getStableCubeSidePrediction(15);

        //sleep(1000);
        sleep(500);

        //Robot drives to the farthest spikemark.
        double driftMod = 0.88;

        //if team element is on the farthest spikemark from the truss.
        //in this case that means left.

        if (cubeSide == CubeSide.Left)
        {
            telemetry.addData("Cube position: ", cubeSide);
            telemetry.update();
            //Drive to the spike mark scoring position
            driveStraight(DRIVE_SPEED, 35, 0);   //Drive 38 in off backboard
            sleep(500); //Sleep half a second
            turnToHeading(TURN_SPEED, 90); // turn to 90 and hold for 0.25 seconds
            holdHeading(TURN_SPEED, 90, 0.25);
            driveStraight(0.2, 8, -90);  //Drive 8 inches
            driveStraight(0.2, -10, -90); // back up 5 inches
            sleep(500); //Sleep half a second

            //Score the purple pixel
            liftDistance(0.5, 7, -1);   //Bring the lift up 5 inches.
            sleep(250);
            spinny.setPower(0.075); //Rotate the end effector to face the ground to score purple pixel.
            sleep(250);
            liftDistance(0.5, 6.75, 1);    //Lower lift down 4 inches to get it as close to ground to score as possible.
            lowerDrop.setPower(.75);    //Drop purple pixel.
            sleep(500);
            spinny.setPower(-.125);
            sleep(500);    //Wait 0.5 seconds.

            driveStraight(DRIVE_SPEED, -5, 1);

            //Drive to backstage scoring position
            turnToHeading(0.25, -90);
            driveStraight(DRIVE_SPEED, 36, -90);
            driveSideways(DRIVE_SPEED, -6, -90);
            sleep(500);

            //Score the yellow pixel
            liftDistance(0.5, 8, -1); //Bring the lift up 8 inches.
            sleep(500);
            spinny.setPower(0.175);
            driveStraight(DRIVE_SPEED, 5, -90);
            upperDrop.setPower(0.75);
            sleep(500);
            liftDistance(0.5, 4, -1);
            driveStraight(DRIVE_SPEED, -4, -90);
            driveSideways(0.5, 42, -90);
        }

        else if (cubeSide == CubeSide.Middle) //If the cube is in the middle spike mark, do the following:
        {
            telemetry.addData("Cube position: ", cubeSide);
            telemetry.update();
            //Drive to the spike mark scoring position
            driveStraight(DRIVE_SPEED, 42, 0);  //Drive forward 42 inches, pushing the blue cube out of the way.
            sleep(250); //Wait 0.5 seconds
            driveStraight(DRIVE_SPEED, -8, 0);  //Drive backwards 8 inches, lining up to score spike mark.

            //Score the purple pixel
            liftDistance(0.5, 7, -1);   //Bring the lift up 5 inches.
            sleep(250);
            spinny.setPower(0.075); //Rotate the end effector to face the ground to score purple pixel.
            sleep(250);
            liftDistance(0.5, 6.75, 1);    //Lower lift down 4 inches to get it as close to ground to score as possible.
            lowerDrop.setPower(.75);    //Drop purple pixel.
            sleep(500);
            spinny.setPower(-.125);
            sleep(500);    //Wait 0.5 seconds.

            //Drive to backstage scoring position
            driveStraight(DRIVE_SPEED, -3, 0);
            turnToHeading(0.25, -90);
            sleep(500);
            driveStraight(DRIVE_SPEED, 40, -90);
            sleep(500);

            //Score the yellow pixel
            liftDistance(0.5, 10, -1); //Bring the lift up 8 inches.
            sleep(500);
            spinny.setPower(0.175);
            sleep(250);
            driveStraight(DRIVE_SPEED, 6, 90);
            sleep(500);
            upperDrop.setPower(0.75);
            sleep(500);
            liftDistance(0.5, 2, -1);
            driveStraight(DRIVE_SPEED, -4, 90);
            driveSideways(0.5, 30, 90);
        }
        else //drive to closest spikemark to truss. Assumes the pixel is on the mark closest to the spikemark.
        {
            telemetry.addData("Cube position: ", cubeSide);
            telemetry.update();
            //Drive to the spike mark scoring position
            //Was 38.
            driveStraight(0.3, 34, 0);   //Drive 38 in off backboard
            sleep(500); //Sleep half a second
            turnToHeading(TURN_SPEED, -90); // turn to 90 and hold for 0.25 seconds
            holdHeading(TURN_SPEED, -90, 0.25);
            driveStraight(0.3, 8, 90);
            driveStraight(0.3, -10, 90);
            turnToHeading(TURN_SPEED, -90); // turn to 90 and hold for 0.25 seconds
            holdHeading(TURN_SPEED, -90, 0.25);
            driveSideways(0.5, -8, 90);
            turnToHeading(TURN_SPEED, -90); // turn to 90 and hold for 0.25 seconds
            holdHeading(TURN_SPEED, -90, 0.25);
            sleep(500); //Sleep half a second

            //Score the purple pixel
            liftDistance(0.5, 7, -1);   //Bring the lift up 5 inches.
            sleep(250);
            spinny.setPower(0.075); //Rotate the end effector to face the ground to score purple pixel.
            sleep(250);
            liftDistance(0.5, 6.75, 1);    //Lower lift down 4 inches to get it as close to ground to score as possible.
            lowerDrop.setPower(.75);    //Drop purple pixel.
            sleep(500);
            spinny.setPower(-.125);
            sleep(500);    //Wait 0.5 seconds.

            //Drive to backstage scoring position

            driveStraight(0.3, 1, 90);
            //turnToHeading(TURN_SPEED, -90);
            //holdHeading(TURN_SPEED,-90, 0.25);
            driveSideways(0.3, -30, 90);
            sleep(500);

            //holdHeading(TURN_SPEED, 90,0.25);
            //sleep(250);
            driveStraight(DRIVE_SPEED, 50.0, 90);
            sleep(250);
            holdHeading(TURN_SPEED, 90, 0.25);
            sleep(250);
            driveSideways(DRIVE_SPEED, 18, 90);
            sleep(250);

            //Score the yellow pixel
            spinny.setPower(0.175);
            liftDistance(0.5, 8, -1); //Bring the lift up 5 inches.
            upperDrop.setPower(0.75);
            sleep(500);
            liftDistance(0.5, 3, -1);
            driveStraight(DRIVE_SPEED, -8, 90);
            driveSideways(DRIVE_SPEED, -26, 90);
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************
    public void colorCheck() {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;



        // Set the LED in the beginning
        colorSensor1.enableLed(bLedOn);
        // check the status of the x button on either gamepad.
        bCurrState = true;

        // check for button state transitions.
        if (bCurrState && (bCurrState != bPrevState)) {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
            colorSensor1.enableLed(bLedOn);
        }

        // update previous state variable.
        bPrevState = bCurrState;

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor1.red() * 8, colorSensor1.green() * 8, colorSensor1.blue() * 8, hsvValues);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        telemetry.update();

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }

    public void liftDistance(double maxDriveSpeed,
                              double distance, int direction) {

        // Ensure that the OpMode is still active
        //if (opModeIsActive()) {

            double averageLiftEncoder = (liftDriveLeft.getCurrentPosition() + liftDriveRight.getCurrentPosition()) / 2;

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * LIFT_COUNTS_PER_INCH);
            int liftDriveLeftTarget = liftDriveLeft.getCurrentPosition() + direction*moveCounts;
            //int liftDriveRightTarget = liftDriveRight.getCurrentPosition() + direction*moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            //liftDriveLeft.setTargetPosition(liftDriveLeftTarget);
            //liftDriveRight.setTargetPosition(liftDriveRightTarget);

            liftDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = abs(maxDriveSpeed);
            //moveRobot(maxDriveSpeed, 0);
           // liftDriveLeft.setPower(maxDriveSpeed);
            //liftDriveRight.setPower(maxDriveSpeed);
            if(direction > 0) {
                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (liftDriveLeft.getCurrentPosition() <= liftDriveLeftTarget)) {
                    liftDriveLeft.setPower((direction) * maxDriveSpeed);
                    liftDriveRight.setPower((direction) * maxDriveSpeed * 0.775);
                    // Apply the turning correction to the current driving speed.
                    // moveRobot(driveSpeed, turnSpeed);

                    // Display drive status for the driver.
                    sendLiftTelemetry();
                }

            }
            else if(direction <0)
            {
                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (liftDriveLeft.getCurrentPosition() >= liftDriveLeftTarget)) {
                    liftDriveLeft.setPower((direction) * maxDriveSpeed);
                    liftDriveRight.setPower((direction) * maxDriveSpeed * 0.775);
                    // Apply the turning correction to the current driving speed.
                    // moveRobot(driveSpeed, turnSpeed);

                    // Display drive status for the driver.
                    sendLiftTelemetry();
                }
            }
            liftDriveLeft.setPower(0);
            liftDriveRight.setPower(0);
            liftDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    //}


    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveSideways(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = FLDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = FRDrive.getCurrentPosition() - moveCounts;
            backLeftTarget = BLDrive.getCurrentPosition() - moveCounts;
            backRightTarget = BRDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FLDrive.setTargetPosition(frontLeftTarget);
            FRDrive.setTargetPosition(frontRightTarget);
            BLDrive.setTargetPosition(backLeftTarget);
            BRDrive.setTargetPosition(backRightTarget);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {

                // Determine required steering to keep on heading
                //turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                //sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = FLDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = FRDrive.getCurrentPosition() + moveCounts;
            backLeftTarget = BLDrive.getCurrentPosition() + moveCounts;
            backRightTarget = BRDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FLDrive.setTargetPosition(frontLeftTarget);
            FRDrive.setTargetPosition(frontRightTarget);
            BLDrive.setTargetPosition(backLeftTarget);
            BRDrive.setTargetPosition(backRightTarget);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                //sendTelemetry(false);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(abs(leftSpeed), abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        FLDrive.setPower(leftSpeed);
        FRDrive.setPower(rightSpeed);
        BLDrive.setPower(leftSpeed);
        BRDrive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos FL:FR:BL:BR",  "%7d:%7d:%7d:%7d",      frontLeftTarget,  frontRightTarget, backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos FL:FR:BL:BR",  "%7d:%7d:%7d:%7d",      FLDrive.getCurrentPosition(), FRDrive.getCurrentPosition(), BLDrive.getCurrentPosition(), BRDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    private void sendLiftTelemetry()
    {
        telemetry.addData("Target Pos LL:LR",  "%7d:%7d",      liftDriveLeftTarget,  liftDriveLeftTarget);
        telemetry.addData("Actual Pos LL:LR",  "%7d:%7d",      liftDriveLeft.getCurrentPosition(), liftDriveRight.getCurrentPosition());
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void dropPixel(){
        //flimsyFlicker.setPower(-1);
        sleep(1000);
    }
}