package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import org.firstinspires.ftc.teamcode.classes.FinalTurretClass;

@TeleOp(name="Tune Turret PD")
public class turretopmode2 extends OpMode {

    private FinalTurretClass turret = new FinalTurretClass();
    private IMU imu;
    DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, flywheel1, flywheel2, intake;


    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    private boolean lastB = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    @Override
    public void init() {

        turret.init(hardwareMap, telemetry);


        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));

        telemetry.addLine("IMU initialized");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {

       // AngularVelocity robotAngVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double robotTurnRateDegPerSec = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        // Z-axis is yaw (heading rotation)
        //double robotTurnRateDegPerSec = robotAngVel.zRotationRate;

        if (gamepad1.b && !lastB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_left && !lastLeft) {
            turret.setP(turret.getKP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_right && !lastRight) {
            turret.setP(turret.getKP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpad_up && !lastUp) {
            turret.setD(turret.getKD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_down && !lastDown) {
            turret.setD(turret.getKD() - stepSizes[stepIndex]);
        }

        lastB = gamepad1.b;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;


        // Pass the turn rate in degrees/sec
        turret.update(true, robotTurnRateDegPerSec);


        telemetry.addData("Status", "Tuning Mode w/ IMU");
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addLine("-------------------");
        telemetry.addData("P Gain", "%.5f", turret.getKP());
        telemetry.addData("D Gain", "%.5f", turret.getKD());
        telemetry.addData("IMU Z Rotation (deg/sec)", "%.2f", robotTurnRateDegPerSec);
        telemetry.update();


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;



        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;




        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        // start it up boi
        telemetry.addData("P Gain", turret.getKP());
        telemetry.addData("D Gain", turret.getKD());
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();

    }
}
