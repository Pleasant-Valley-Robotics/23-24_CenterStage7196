package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "V1.0.0", group = "Iterative Opmode")
public class DriveTesitng extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        DcMotor FLDrive = null; // FLDrive is in port 0 of CH
        DcMotor FRDrive = null; // FRDrive is in port 1 of CH
        DcMotor BLDrive = null; // BLDrive is in port 2 of CH
        DcMotor BRDrive = null; // BRDrive is in port 3 of CH
        //* TODO uncomment this code for defining liftDRiveLeft, liftDriveRight, and DroneLauncher
        //DcMotor liftDriveLeft = null; // liftDriveLeft is in port 0 of EH
        //DcMotor liftDriveRight = null; // liftDriveRight is in port 1 of EH
        //CRServo droneLaunch = null;
        double turnMovement = 0;
        double strafeMovement = 0;
        double straightMovement = 0;

        //Write numerical variables here
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        //* TODO uncomment this code for hardware mapping liftDRiveLeft, liftDriveRight, and DroneLauncher
        // liftDrive = hardwareMap.get(DcMotor.class, "liftDrive");
        // liftJoint = hardwareMap.get(DcMotor.class, "liftJoint");
        //droneLaunch = hardwareMap.get(CRServo.class, "droneLaunch");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        //* TODO uncomment this code for setting direction liftDRiveLeft, liftDriveRight, and DroneLauncher
        //  liftDrive.setDirection(DcMotor.Direction.FORWARD);
        //   liftJoint.setDirection(DcMotor.Direction.REVERSE);
        //droneLaunch.setDirection(CRServo.Direction.FORWARD);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //* TODO uncomment this code for defining ZeroPowerBehavior liftDRiveLeft, liftDriveRight, and DroneLauncher
        // liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //  liftJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
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

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double straightMovement = gamepad1.left_stick_y;
            //double strafeMovement = -gamepad1.left_stick_x;
            //double turnMovement = -gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                turnMovement = gamepad1.right_stick_x * 0.5;
                strafeMovement = gamepad1.left_stick_x * 0.5;
                straightMovement = -gamepad1.left_stick_y * 0.5;
            } else {
                turnMovement = gamepad1.right_stick_x;
                strafeMovement = gamepad1.left_stick_x;
                straightMovement = -gamepad1.left_stick_y;
            }

            /*if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }*/


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
            telemetry.update();

            // For analog
            //*TODO Commented out liftDriveLeft code
            /*
            double liftJoystick = gamepad2.right_stick_y;
            if (liftJoystick > 1) {
                liftJoystick = 1.0;
            }
            if (liftJoystick > 0.05 || liftJoystick < -0.05) {
                liftDriveRight.setPower(liftJoystick * 0.4);
                liftDriveLeft.setPower(liftJoystick * 0.4);
            } else {
                liftDriveRight.setPower(0);
                liftDriveLeft.setPower(0);
            }
             */
            //* TODO delete this comment for DroneLauncher
            /*if (gamepad2.right_trigger > 0.1) {
                droneLaunch.setPower(0);
            }
            else
            {
                droneLaunch.setPower(-1.3);
            }
             */

            //* TODO uncomment this code and reprogram for joint move that is now liftDiveRight
            //analog
          /*  double jointMove = gamepad2.left_stick_y;
            if (jointMove > 1.0) {
                jointMove = 1.0;
            }
            if (jointMove > 0.05 || jointMove < -0.05) {
                //liftJoint.setPower(jointMove * 0.4);
            } else {
              //  liftJoint.setPower(-0.05);
            }
        }
    }

/*
    private void initTfod() {
        final boolean USE_WEBCAM = true;
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Microsoft Camera 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
    */
        }
    }
}