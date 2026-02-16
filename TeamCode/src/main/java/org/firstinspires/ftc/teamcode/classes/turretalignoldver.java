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

    // --- MECHANICAL MATH ---
    // Motor is 384.5 ticks per rev.
    // Gear ratio is 210:70 (which is 3:1).
    // Total ticks for one turret rotation = 384.5 * 3 = 1153.5.
    // Ticks per degree = 1153.5 / 360 = 3.204.
    private static final double TICKS_PER_DEGREE = 3.204;

    // Set these by moving the turret by hand and checking telemetry!
    private static final int LEFT_LIMIT = -1000;
    private static final int RIGHT_LIMIT = 1000;

    // --- TUNING ---
    private static final double KP = 0.035;      // How "aggressive" the tracking is
    private static final double TOLERANCE = 1.0; // Stop if within 1 degree
    private static final double MAX_POWER = 0.4; // Don't go faster than 40% speed
    private static final double SEARCH_POWER = 0.2; // Speed while looking for target

    private boolean searchingRight = true;

    public turretalignoldver(HardwareMap hardwareMap, Telemetry telemetry) {
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

        // P-LOOP: Power = Error (tx) times our Tuning Constant (KP)
        double power = tx * KP;

        // If we are close enough, just stop
        if (Math.abs(tx) < TOLERANCE) {
            power = 0;
        }

        // SAFETY: If we are at the left limit and trying to go further left, stop.
        if (currentPos <= LEFT_LIMIT && power < 0) {
            power = 0;
        }
        // SAFETY: If we are at the right limit and trying to go further right, stop.
        if (currentPos >= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        // Clamp power so it doesn't exceed our MAX_POWER
        turretMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
        telemetry.addData("Status", "Tracking Target");
    }

    private void searchForTarget() {
        int currentPos = turretMotor.getCurrentPosition();

        // If we hit the far right, start going left
        if (currentPos >= RIGHT_LIMIT) {
            searchingRight = false;
        }

        // If we hit the far left, start going right
        if (currentPos <= LEFT_LIMIT) {
            searchingRight = true;
        }

        // Decide power based on our direction boolean
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