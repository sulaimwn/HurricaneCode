package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.classes.TurretMechanism;

@TeleOp(name="Tune Turret PD")
public class TurretOpMode extends OpMode {

    DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, flywheel1, flywheel2, intake;
    private TurretMechanism turret = new TurretMechanism();
    private Limelight3A limelight;

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret.init(hardwareMap, telemetry);
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.start();

    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {
        // Adjust precision with 'B'
        if (gamepad1.b) {
            // In a real OpMode, we'd use a debounce, but keeping it simple for now
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // D-pad left/right adjusts the P gain
        if (gamepad1.dpad_left) {
            turret.setP(turret.getKP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_right) {
            turret.setP(turret.getKP() + stepSizes[stepIndex]);
        }

        // D-pad up/down adjusts the D gain
        if (gamepad1.dpad_up) {
            turret.setD(turret.getKD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_down) {
            turret.setD(turret.getKD() - stepSizes[stepIndex]);
        }


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

        turret.update(true);


        telemetry.addData("P Gain", turret.getKP());
        telemetry.addData("D Gain", turret.getKD());
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();
    }


}