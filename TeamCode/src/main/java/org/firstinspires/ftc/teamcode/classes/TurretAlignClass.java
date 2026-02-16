package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretAlignClass {
    private Limelight3A limelight;
    private DcMotorEx turret;

    private double kP = 0.035; // Adjusted for Limelight degrees
    private double kD = 0.002;
    private double goalX = 0;

    private double lastError = 0;
    private double angleTolerance = 0.5;
    private final double MAX_POWER = 0.6;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setP(double newKP) {
        kP = newKP;
    }

    public double getKP() {
        return kP;
    }

    public void setD(double newKD) {
        kD = newKD;
    }

    public double getKD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(LLResult curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (curID == null || !curID.isValid()) {
            turret.setPower(0);
            lastError = 0;
            return;
        }

        // Valid ID, start PD controller
        // tx is the bearing (horizontal offset)
        double error = goalX - curID.getTx();
        double pTerm = error * kP;

        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;

            if (Math.abs(error) < angleTolerance) {
                power = 0;
            } else {
                power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
            }
        }

        turret.setPower(power);
        lastError = error;
    }
}