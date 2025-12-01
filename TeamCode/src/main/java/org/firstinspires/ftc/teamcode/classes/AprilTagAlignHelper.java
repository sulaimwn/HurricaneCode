package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class AprilTagAlignHelper {
    private Limelight3A limelight;
    private IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Telemetry telemetry;

    // PID constants
    public static  double kP = 0.04;
    public static  double MIN_POWER = 0.05;
    public static  double TX_TOLERANCE = 1.0;

    public AprilTagAlignHelper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public double getRotationCorrection() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("AT-Status", "No Target");
            return 0.0;
        }

        double tx = result.getTx();
        telemetry.addData("tx", tx);

        if (Math.abs(tx) <= TX_TOLERANCE) {
            telemetry.addData("AT-Status", "Aligned");
            return 0.0;
        }

        double power = kP * tx;

        if (Math.abs(power) < MIN_POWER)
            power = Math.copySign(MIN_POWER, power);

        return Math.max(-1.0, Math.min(1.0, power));
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