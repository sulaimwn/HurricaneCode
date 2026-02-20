package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalTurretClass {

    private DcMotorEx turret;
    private Limelight3A limelight;

    private double kP = 0.01;
    private double kD = 0.001;
    private double kF = 0.0;

    private double lastError = 0;
    private double filteredError = 0;
    private double alpha = 0.2;

    private double angleTolerance = 0.2;
    private final double MAX_POWER = 1.0;
    private final double MIN_POWER = 0.05;

    private final ElapsedTime timer = new ElapsedTime();

    // Encoder limits
    private final int LEFT_LIMIT = -600;
    private final int RIGHT_LIMIT = 600;

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        // switch to the first pipeline
        limelight.start();

        telemetry.addLine("Turret Mechanism Initialized");
        timer.reset();
    }


    public void setP(double newKP) { kP = newKP; }
    public void setD(double newKD) { kD = newKD; }
    public void setF(double newKF) { kF = newKF; }

    public double getKP() { return kP; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }

    public void resetTimer() {
        timer.reset();
    }


    public void update(boolean isAligning, double robotAngularVelocity) {

        if (!isAligning) {
            turret.setPower(0);
            lastError = 0;
            timer.reset();
            return;
        }

        LLResult llResult = limelight.getLatestResult();

        double deltaTime = timer.seconds();
        timer.reset();

        if (deltaTime < 0.001) {
            deltaTime = 0.001;
        }

        if (llResult == null || !llResult.isValid()) {
            turret.setPower(0);
            lastError = 0;
            return;
        }


        double rawError = llResult.getTx();
        filteredError = alpha * rawError + (1 - alpha) * filteredError;
        double error = filteredError;

        if (Math.abs(error) < angleTolerance) {
            turret.setPower(0);
            lastError = error;
            return;
        }

        double pTerm = error * kP;
        double dTerm = ((error - lastError) / deltaTime) * kD;

        double fTerm = -robotAngularVelocity * kF;

        double power = pTerm + dTerm + fTerm;

        if (Math.abs(power) > 0.001) {
            power += Math.signum(power) * MIN_POWER;
        }

        power = Range.clip(power, -MAX_POWER, MAX_POWER);


        int position = turret.getCurrentPosition();

        if (position > RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        if (position < LEFT_LIMIT && power < 0) {
            power = 0;
        }

        turret.setPower(power);

        lastError = error;
    }
}
