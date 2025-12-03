package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive")
public class drivecode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor ShooterMotor1 = null;
    private DcMotor ShooterMotor2 = null;
    double speedLimiter = 1.65;
    double Voltage;
    double shooterPower = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        ShooterMotor1 = hardwareMap.get(DcMotor.class, "ShooterMotor1");
        ShooterMotor2 = hardwareMap.get(DcMotor.class, "ShooterMotor2");

//         For Skunkworks
//        FL.setDirection(DcMotor.Direction.REVERSE);
//        BL.setDirection(DcMotor.Direction.REVERSE);
//        FR.setDirection(DcMotor.Direction.FORWARD);
//        BR.setDirection(DcMotor.Direction.FORWARD);

        // For basic bot
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        ShooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

//        //darkstar
//        FL.setDirection(DcMotor.Direction.FORWARD);
//        BL.setDirection(DcMotor.Direction.FORWARD);
//        FR.setDirection(DcMotor.Direction.REVERSE);
//        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double voltage = Double.POSITIVE_INFINITY;

            // Loop through all voltage sensors (some hubs report multiple)
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double sensorVoltage = sensor.getVoltage();
                if (sensorVoltage > 0) {
                    voltage = Math.min(voltage, sensorVoltage);
                }
            }

            // Display voltage on Driver Station telemetry
            telemetry.addData("Battery Voltage", "%.2f V", voltage);

            if (gamepad2.dpad_down) {
                speedLimiter = 2;
            } else if (gamepad2.dpad_up) {
                speedLimiter = 1;
            } else if (gamepad2.dpad_right) {
                speedLimiter = 1.65;
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Skunkwork v2
            double axial = gamepad2.left_stick_y;  // Note: pushing stick forward gives negative value ( strafe froward/backward
            double lateral = -gamepad2.left_stick_x * 1.1; //strafe right/left
            double yaw = gamepad2.right_stick_x; //turning one place

            //darkstar
//            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//            double lateral = -gamepad1.left_stick_x * 1.1;
//            double yaw = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontPower /= speedLimiter;
            rightFrontPower /= speedLimiter;
            leftBackPower /= speedLimiter;
            rightBackPower /= speedLimiter;

            // Send calculated power to wheels
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);


            // SHOOTER 1

            if (gamepad1.right_trigger > 0.05) {
                shooterPower += 0.01;

                if (shooterPower > 1) shooterPower = 1;

            } else {
                shooterPower -= 0.01;
                if (shooterPower < 0) shooterPower = 0;
            }


            if (gamepad1.right_bumper) {
                ShooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                ShooterMotor1.setPower(0.7);
            } else {
                ShooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
                ShooterMotor1.setPower(shooterPower);

            }

            // SHOOTER MOTOR 2
            if (gamepad1.left_bumper) {
                ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                ShooterMotor2.setPower(0.7);
            } else if (gamepad1.left_trigger > 0.05) {
                ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                ShooterMotor2.setPower(gamepad1.left_trigger);
            } else {
                ShooterMotor2.setPower(0);

            }


            // TRIGGER ALT CODE INCASE

            //double shooterPower1 = gamepad1.right_trigger; //motor 1 uses right trigger
            // double shooterPower2 = gamepad1.left_trigger; //motor 2 uses left trigger


            //full power if they are pressed
            // if (gamepad1.right_bumper) shooterPower1 = 1;
            // if (gamepad1.left_bumper) shooterPower2 = 1;

            // ShooterMotor1.setPower(shooterPower1);
            // ShooterMotor2.setPower(shooterPower2);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Current speedLimiter: ", speedLimiter);
            telemetry.update();
        }
    }
}

