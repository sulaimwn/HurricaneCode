package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AprilTagAlignHelper {
    private Limelight3A limelight;
    private IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, TurretMotor;
    private Telemetry telemetry;

    // PID constants
    private static final double kP = 0.04;
    private static final double MIN_POWER = 0.05;
    private static final double TX_TOLERANCE = 1.0;

    public AprilTagAlignHelper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void alignToAprilTag(boolean isAligning) {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            telemetry.addData("tx", tx);

            if (isAligning) {
                double rotationPower = kP * tx;

                if (Math.abs(rotationPower) < MIN_POWER && Math.abs(tx) > TX_TOLERANCE) {
                    rotationPower = Math.copySign(MIN_POWER, rotationPower);
                }

                if (Math.abs(tx) <= TX_TOLERANCE) {
                    stopMotors();
                    telemetry.addData("status", "Aligned");
                } else {
                    rotationPower = Math.max(-1.0, Math.min(1.0, rotationPower));
                    rotateInPlace(rotationPower);
                    telemetry.addData("status", "Rotating");
                    telemetry.addData("rotationPower", rotationPower);
                }
            } else {
                stopMotors();
            }
        } else {
            stopMotors();
            telemetry.addData("status", "No target");
        }
    }

    private void rotateInPlace(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
