package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import android.drm.DrmStore;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp(name = "Teleop New Robot", group = "Iterative Opmode")
public class RobotAlpha2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        DcMotor FLDrive = null; // FLDrive is in port 0 of CH
        DcMotor FRDrive = null; // FRDrive is in port 1 of CH
        DcMotor BLDrive = null; // BLDrive is in port 2 of CH
        DcMotor BRDrive = null; // BRDrive is in port 3 of CH
        CRServo spinny = null; // spinny is in port 0 of CH
        CRServo upperDrop = null; // upperDrop is in port 1 of CH
        CRServo lowerDrop = null; // lowerDrop is in port 2 of CH
        //* TODO uncomment this code for defining DroneLauncher.
        DcMotor liftDriveLeft = null; // liftDriveLeft is in port 0 of EH
        DcMotor liftDriveRight = null; // liftDriveRight is in port 1 of EH
        DcMotor spintake = null; // spintake is in port 2 of EH
        CRServo droneLaunch = null;
        CRServo droneRotate = null;
        double turnMovement = 0;
        double strafeMovement = 0;
        double straightMovement = 0;

        //Write numerical variables here
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        spinny = hardwareMap.get(CRServo.class, "spinny");
        upperDrop = hardwareMap.get(CRServo.class, "upperDrop");
        lowerDrop = hardwareMap.get(CRServo.class, "lowerDrop");
        spintake = hardwareMap.get(DcMotor.class, "spintake");
        //* TODO uncomment this code for hardware mapping DroneLauncher.
         liftDriveLeft = hardwareMap.get(DcMotor.class, "liftDriveLeft");
         liftDriveRight = hardwareMap.get(DcMotor.class, "liftDriveRight");
        droneLaunch = hardwareMap.get(CRServo.class, "droneLaunch");
        droneRotate = hardwareMap.get(CRServo.class, "droneRotate");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        spinny.setDirection(DcMotorSimple.Direction.FORWARD);
        upperDrop.setDirection(CRServo.Direction.FORWARD);
        lowerDrop.setDirection(CRServo.Direction.FORWARD);
        spintake.setDirection(DcMotor.Direction.FORWARD);
        liftDriveLeft.setDirection(DcMotor.Direction.REVERSE);
        liftDriveRight.setDirection(DcMotor.Direction.REVERSE);
        droneLaunch.setDirection(CRServo.Direction.FORWARD);
        droneRotate.setDirection(CRServo.Direction.FORWARD);

        //Set the zero
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        liftDriveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();


        waitForStart();
        double yHeading = 0;
        double xHeading = -90;
        double bHeading = 90;
        double aHeading = 180;

        while (opModeIsActive()) {
            double maxWheelPower;

            //if the gamepad 1s right bumper is clicked, set every type of movement to their corresponding joysticks. and give them
            //more power.
            if (gamepad1.right_bumper) {
                turnMovement = gamepad1.right_stick_x * 0.5;
                strafeMovement = gamepad1.left_stick_x * 0.5;
                straightMovement = -gamepad1.left_stick_y * 0.5;
            }
            //else set them with their regular power.
            else {
                turnMovement = gamepad1.right_stick_x;
                strafeMovement = gamepad1.left_stick_x;
                straightMovement = -gamepad1.left_stick_y;
            }

            //COLE codes spintake
            // Good job looks good - Oliver
  /*        if (gamepad1.a)
            {
                spintake.setPower(-1);
            }
            else if (gamepad1.b)
            {
                spintake.setPower(1);
            }
            else
            {
                spintake.setPower(0);
            }
*/
            // Cole your code is respectfully denied for spintake but slide is muy bien
            if (gamepad1.left_trigger > 0.05) // if a is NOT pressed AND b are pressed then set power to 1 and open drops
            {
                spintake.setPower(1);
                upperDrop.setPower(0.5);
                lowerDrop.setPower(0.85);
            }
            else if (gamepad1.right_trigger > 0.05 && gamepad1.left_trigger < 0.05) // if a AND b NOT presses set power 0
            {
                spintake.setPower(-1);
                if (gamepad2.a) // if a pressed set drops to be open
                {
                    {
                        upperDrop.setPower(0.5);
                        lowerDrop.setPower(0.85);
                    }
                } else if (!gamepad2.b) // if a is not pressed set upper drop to closed
                {
                    upperDrop.setPower(-0.2);
                }

                //When B is pressed toggles the position of lowerDrop
                if (gamepad2.b) // if b pressed open lower drop
                {
                    {
                        lowerDrop.setPower(0.85);
                    }
                } else if (!gamepad2.b && !gamepad2.a) // if 2b AND 2a AND 1b NOT pressed set lower drop to closed
                {
                    lowerDrop.setPower(0.2);
                }
            }
            else
            {
                spintake.setPower(0);
                if (gamepad2.a) // if a pressed set drops to be open
                {
                    {
                        upperDrop.setPower(0.5);
                        lowerDrop.setPower(0.85);
                    }
                } else if (!gamepad2.b) // if a is not pressed set upper drop to closed
                {
                    upperDrop.setPower(-0.2);
                }

                //When B is pressed toggles the position of lowerDrop
                if (gamepad2.b) // if b pressed open lower drop
                {
                    {
                        lowerDrop.setPower(0.85);
                    }
                } else if (!gamepad2.b && !gamepad2.a && gamepad1.left_trigger < 0.05) // if 2b AND 2a AND spintake NOT pressed set lower drop to closed
                {
                    lowerDrop.setPower(0.2);
                }
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double FLPower = straightMovement + strafeMovement + turnMovement;
            double FRPower = straightMovement - strafeMovement - turnMovement;
            double BLPower = straightMovement - strafeMovement + turnMovement;
            double BRPower = straightMovement + strafeMovement - turnMovement;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            maxWheelPower = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            maxWheelPower = Math.max(maxWheelPower, Math.abs(BLPower));
            maxWheelPower = Math.max(maxWheelPower, Math.abs(BRPower));

            //if the maximum wheel power is greater than 1 set each motors power to themselves/the maximum wheel power.
            if (maxWheelPower > 1.0) {
                FLPower /= maxWheelPower;
                FRPower /= maxWheelPower;
                BLPower /= maxWheelPower;
                BRPower /= maxWheelPower;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            FLPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            BLPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            FRPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            BRPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            FLDrive.setPower(FLPower);
            FRDrive.setPower(FRPower);
            BLDrive.setPower(BLPower);
            BRDrive.setPower(BRPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", FLPower, FRPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BLPower, BRPower);
            telemetry.addData("UpperPower: ", upperDrop.getPower());
            telemetry.addData("LowerPower: ", lowerDrop.getPower());
            telemetry.addData("SpinTakePower: ", spintake.getPower());
            telemetry.addData("liftJoystick: ", liftDriveLeft.getPower());
            telemetry.addData("Left lift position: ", String.valueOf(liftDriveLeft.getCurrentPosition()));
            telemetry.addData("Right lift position: ", String.valueOf(liftDriveRight.getCurrentPosition()));
            telemetry.update();

            double liftJoystick = gamepad2.left_stick_y;
            //if the joystick for the servo flipping the blue 3d printed part is at a power of more than 1, reset it to the
            //maximum power allowed to keep it in bounds.
            if (liftJoystick > 1) {
                liftJoystick = 1.0;
            }
            if (liftJoystick > 0.05 || liftJoystick < -0.05) {
                liftDriveLeft.setPower(liftJoystick);
                liftDriveRight.setPower(liftJoystick);
            } else {
                liftDriveLeft.setPower(0);
                liftDriveRight.setPower(0);
            }



            // furthest 5850

            //higher encoder.
            //highest it csan go.
//          if (liftDriveRight.getCurrentPosition() <= 5850 && liftDriveLeft.getCurrentPosition() >= -5850)
//          //encoders on the slide are cursed so keep these values unless they break
//           {
//                if (liftJoystick > 0.05)
//                {
//                        liftDriveLeft.setPower(-liftJoystick);
//                        liftDriveRight.setPower(-liftJoystick);
//                }
//                else{
//                    liftDriveLeft.setPower(0);
//                    liftDriveRight.setPower(0);
//                }
//            }
//          //lowest encoder.
//          //lowest it can go.
//           else if (liftDriveRight.getCurrentPosition() >= 0 && liftDriveLeft.getCurrentPosition() <= 0)
//           {
//                if (liftJoystick < -0.05)
//                {
//                    liftDriveLeft.setPower(-liftJoystick);
//                    liftDriveRight.setPower(-liftJoystick);
//                }
//                else{
//                    liftDriveLeft.setPower(0);
//                    liftDriveRight.setPower(0);
//                }
//            }


            //-1 power for having it as far in as possible.
            //1 for having it as far out as possible.

            if (gamepad2.left_bumper == true)
            {
                spinny.setPower(0.2);
            }
            else
            {
                spinny.setPower(0.55); // for auto -0.35
            }

            //* TODO delete this comment for DroneLauncher
            //Changed where drone launch was. It's now the right trigger on gamepad2.
            if (gamepad2.right_trigger > 0.05) {
                droneLaunch.setPower(-.1);
            }
            else
            {
                droneLaunch.setPower(-1.3);
            }
            if (gamepad2.y) {
                droneRotate.setPower(-.07);
            }
            else
            {
                droneRotate.setPower(-.45);
            }

            //Ask Oliver if we can do this feasibly.
            //if we have no buttons left to bind then we can use gamepad2 right trigger and left trigger for Dereks request.
            //set spinny to 3 different positions based on a button press so lower pixels can be scored and moved.

        }
    }
}