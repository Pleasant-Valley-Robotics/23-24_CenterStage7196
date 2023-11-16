package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.signum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "V1.1.2", group = "Iterative Opmode")
public class RobotAlpha extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        DcMotor FLDrive = null; // standard motor declarations
        DcMotor FRDrive = null;
        DcMotor BLDrive = null;
        DcMotor BRDrive = null;
        DcMotor liftDrive = null;
        DcMotor liftJoint = null;
        DcMotor leftActuator = null;
        DcMotor rightActuator = null;
        ColorSensor colorSensor = null;

        CRServo droneLaunch = null;
        CRServo claw = null;

        //Write numerical variables here
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        liftDrive = hardwareMap.get(DcMotor.class, "liftDrive");
        liftJoint = hardwareMap.get(DcMotor.class, "liftJoint");
        leftActuator = hardwareMap.get(DcMotor.class, "leftActuator");
        rightActuator = hardwareMap.get(DcMotor.class, "rightActuator");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        droneLaunch = hardwareMap.get(CRServo.class, "droneLaunch");
        claw = hardwareMap.get(CRServo.class, "claw");

        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);
        liftJoint.setDirection(DcMotor.Direction.REVERSE);
        leftActuator.setDirection(DcMotor.Direction.FORWARD);
        rightActuator.setDirection(DcMotor.Direction.FORWARD);

        droneLaunch.setDirection(CRServo.Direction.FORWARD);
        claw.setDirection(CRServo.Direction.FORWARD);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        while (opModeIsActive())
        {
            double maxWheelPower;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double straightMovement = gamepad1.left_stick_y;
            double strafeMovement = gamepad1.left_stick_x;
            double turnMovement = gamepad1.right_stick_x;

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

            if (maxWheelPower > 1.0)
            {
                FLPower /= maxWheelPower;
                FRPower /= maxWheelPower;
                BLPower /= maxWheelPower;
                BRPower /= maxWheelPower;
            }

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
            double liftJoystick = gamepad2.left_stick_y;
            if (liftJoystick > 1)
            {
                liftJoystick = 1.0;
            }
            if (liftJoystick > 0.05 || liftJoystick < -0.05)
            {
                liftDrive.setPower(liftJoystick * 0.4);
            }
            else
            {
                liftDrive.setPower(0);
            }

            boolean actuatorMoveUp = gamepad2.dpad_up;
            boolean actuatorMoveDown = gamepad2.dpad_down;

            if (actuatorMoveUp && !actuatorMoveDown)
            {
                leftActuator.setPower(0.7);
                rightActuator.setPower(0.7);
            }
            else if (actuatorMoveDown && !actuatorMoveUp)
            {
                leftActuator.setPower(-0.7);
                rightActuator.setPower(-0.7);
            }
            else
            {
                leftActuator.setPower(0);
                rightActuator.setPower(0);
            }

            boolean clawOpen = gamepad2.a;
            boolean clawClosed = gamepad2.b;
            //While holding A, hold the pixel
            if (gamepad2.a)
            {
                claw.setPower(0.9);
            }
            //When let go of A, let go of pixel
            else if (gamepad2.b)
            {
                claw.setPower(1);
            }

            if (gamepad2.right_trigger > 0.1) {
                droneLaunch.setPower(0);
            }
            else
            {
                droneLaunch.setPower(-1.3);
            }

            //analog
            double jointMove = gamepad2.right_stick_y;
            if (jointMove > 1.0)
            {
                jointMove = 1.0;
            }
            if (jointMove > 0.05 || jointMove < -0.05)
            {
                liftJoint.setPower(jointMove * 0.4);
            }
            else
            {
                liftJoint.setPower(0);
            }
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Lift joint encoder value  ", liftJoint.getCurrentPosition());
            telemetry.addData("Lift drive encoder value  ", liftDrive.getCurrentPosition());
        }
    }
}