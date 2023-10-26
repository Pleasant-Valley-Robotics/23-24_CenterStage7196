package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Dumb_Auto_11_Blue")
public class Dumb_Auto_11B extends LinearOpMode{
    /* Declare OpMode members. */
    DcMotor FLDrive = null; // standard motor declarations
    DcMotor FRDrive = null;
    DcMotor BLDrive = null;
    DcMotor BRDrive = null;
    IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                FLDrive.getCurrentPosition(),
                BLDrive.getCurrentPosition(),
                FRDrive.getCurrentPosition(),
                BRDrive.getCurrentPosition()
                );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

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
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBLeftTarget;
        int newBRightTarget;
        int newFLeftTarget;
        int newFRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBLeftTarget = BLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRightTarget = BRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFLeftTarget = FLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFRightTarget = FRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            BLDrive.setTargetPosition(newBLeftTarget);
            BRDrive.setTargetPosition(newBRightTarget);
            FLDrive.setTargetPosition(newFLeftTarget);
            FRDrive.setTargetPosition(newFRightTarget);

            // Turn On RUN_TO_POSITION
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            BLDrive.setPower(Math.abs(speed));
            BRDrive.setPower(Math.abs(speed));
            FLDrive.setPower(Math.abs(speed));
            FRDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BLDrive.isBusy() && BRDrive.isBusy() && FRDrive.isBusy() && FLDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFLeftTarget,  newFRightTarget, newBLeftTarget,
                        newBRightTarget );
                telemetry.addData("Currently at",  " at %7d :%7d",
                        FLDrive.getCurrentPosition(), BLDrive.getCurrentPosition(),
                        BRDrive.getCurrentPosition(), FRDrive.getCurrentPosition());
                telemetry.update();
            }

            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
