package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test Auto V1.0.0")
public class Dumb_Auto_11B extends LinearOpMode {
    /* Declare OpMode members. */

    // Declare motor and sensor variables
    DcMotor FLDrive = null; // Front Left Drive Motor
    DcMotor FRDrive = null; // Front Right Drive Motor
    DcMotor BLDrive = null; // Back Left Drive Motor
    DcMotor BRDrive = null; // Back Right Drive Motor
    IMU imu = null; // Inertial Measurement Unit
    private ElapsedTime runtime = new ElapsedTime(); // Timer for tracking time

    // Constants for calculating encoder counts and speed
    static final double COUNTS_PER_MOTOR_REV = 28; // Encoder counts per motor revolution
    static final double DRIVE_GEAR_REDUCTION = 20; // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 96/25.4; // Diameter of the robot's wheels in mm/(mm/in)
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.

        // Map the motors to the hardware configuration
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        // Set motor directions and behaviors
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and set motor run modes
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "\nFL:%7d FR:%7d\nBL:%7d BR:%7d",
                FLDrive.getCurrentPosition(), BLDrive.getCurrentPosition(),
                FRDrive.getCurrentPosition(), BRDrive.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Move the robot using the encoderDrive method
        encoderDrive(0.7,  30,  30, 30, 30);
        encoderDrive(-0.7,  28,  28, 28, 28);
        encoderTurn(0.3, 90);
        encoderDrive(0.7, 95, 95, 95, 95);


        // Display a message when the path is complete
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double BleftInches, double BrightInches, double FleftInches, double FrightInches) {
        int newBLeftTarget;
        int newBRightTarget;
        int newFLeftTarget;
        int newFRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target positions for the motors and pass them to the motor controller
            newBLeftTarget = BLDrive.getCurrentPosition() + (int)(BleftInches * COUNTS_PER_INCH);
            newBRightTarget = BRDrive.getCurrentPosition() + (int)(BrightInches * COUNTS_PER_INCH);
            newFLeftTarget = FLDrive.getCurrentPosition() + (int)(FleftInches * COUNTS_PER_INCH);
            newFRightTarget = FRDrive.getCurrentPosition() + (int)(FrightInches * COUNTS_PER_INCH);

            BLDrive.setTargetPosition(newBLeftTarget);
            BRDrive.setTargetPosition(newBRightTarget);
            FLDrive.setTargetPosition(newFLeftTarget);
            FRDrive.setTargetPosition(newFRightTarget);

            // Turn On RUN_TO_POSITION mode for the motors
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion
            runtime.reset();
            BLDrive.setPower(speed);
            BRDrive.setPower(speed);
            FLDrive.setPower(speed);
            FRDrive.setPower(speed);

            // Keep looping while we are still active, there is time left, and both motors are running.
            // The move will stop when any of these conditions are met.
            while (opModeIsActive()) {

                // Display target and current position for the driver
                telemetry.addData("Running to",  "\nFL:%7d FR:%7d\nBL:%7d BR:%7d",
                        newFLeftTarget, newFRightTarget,
                        newBLeftTarget, newBRightTarget );
                telemetry.addData("Currently at",  "\nFL:%7d FR:%7d\nBL:%7d BR:%7d",
                        FLDrive.getCurrentPosition(), FRDrive.getCurrentPosition(),
                        BLDrive.getCurrentPosition(), BRDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop the motors once the move is complete
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            // Turn off RUN_TO_POSITION mode
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // Optional pause after each move.
        }
    }

    public void encoderTurn(double speed,
                             int degrees) {
        int newLeftTarget;
        int newRightTarget;

        final double DEGREES_TO_TICKS = 78.15;

        newLeftTarget = (int)(DEGREES_TO_TICKS * degrees);
        newRightTarget = (int)(DEGREES_TO_TICKS * degrees);

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target positions for the motors and pass them to the motor controller


            BLDrive.setTargetPosition(newLeftTarget);
            BRDrive.setTargetPosition(newRightTarget);
            FLDrive.setTargetPosition(newLeftTarget);
            FRDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION mode for the motors
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion
            runtime.reset();
            BLDrive.setPower(speed);
            BRDrive.setPower(speed);
            FLDrive.setPower(speed);
            FRDrive.setPower(speed);

            // Keep looping while we are still active, there is time left, and both motors are running.
            // The move will stop when any of these conditions are met.
            while (opModeIsActive()) {

                // Display target and current position for the driver
                telemetry.addData("Running to",  "\nFL:%7d FR:%7d\nBL:%7d BR:%7d",
                        newLeftTarget, newRightTarget,
                        newLeftTarget, newRightTarget);
                telemetry.addData("Currently at",  "\nFL:%7d FR:%7d\nBL:%7d BR:%7d",
                        FLDrive.getCurrentPosition(), BLDrive.getCurrentPosition(),
                        BRDrive.getCurrentPosition(), FRDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop the motors once the move is complete
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            // Turn off RUN_TO_POSITION mode
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // Optional pause after each move.
        }
    }
}

