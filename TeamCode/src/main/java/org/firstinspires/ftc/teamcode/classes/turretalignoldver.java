package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class turretalignoldver {

    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private Telemetry telemetry;


    private static final double TICKS_PER_DEGREE = 3.204;

    private static final int LEFT_LIMIT = -1000;
    private static final int RIGHT_LIMIT = 1000;

    // --- TUNING ---
    private static final double KP = 0.035;
    private static final double TOLERANCE = 1.0;
    private static final double MAX_POWER = 0.4;
    private static final double SEARCH_POWER = 0.2;

    private boolean searchingRight = true;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder to 0 when the robot starts
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update(boolean active) {
        if (!active) {
            turretMotor.setPower(0);
            return;
        }

        LLResult result = limelight.getLatestResult();

        // Check if Limelight actually sees a target
        if (result != null && result.isValid()) {
            trackTarget(result.getTx());
        } else {
            searchForTarget();
        }

        sendTelemetry();
    }

    private void trackTarget(double tx) {
        int currentPos = turretMotor.getCurrentPosition();

        double power = tx * KP;

        // If we are close enough, just stop
        if (Math.abs(tx) < TOLERANCE) {
            power = 0;
        }

        if (currentPos <= LEFT_LIMIT && power < 0) {
            power = 0;
        }
        if (currentPos >= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        turretMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
        telemetry.addData("Status", "Tracking Target");
    }

    private void searchForTarget() {
        int currentPos = turretMotor.getCurrentPosition();

        if (currentPos >= RIGHT_LIMIT) {
            searchingRight = false;
        }

        if (currentPos <= LEFT_LIMIT) {
            searchingRight = true;
        }

        double power;
        if (searchingRight == true) {
            power = SEARCH_POWER;
        } else {
            power = -SEARCH_POWER;
        }

        turretMotor.setPower(power);
        telemetry.addData("Status", "Searching...");
    }

    private void sendTelemetry() {
        telemetry.addData("Turret Pos", turretMotor.getCurrentPosition());
        telemetry.update();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}