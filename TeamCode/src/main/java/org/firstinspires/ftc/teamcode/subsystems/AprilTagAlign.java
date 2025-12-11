package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public enum AprilTagAlign implements Subsystem {
    INSTANCE;

    private Limelight3A limelight;
    private MotorEx fl, fr, bl, br;
    private Telemetry telemetry;

    private static final double kP = 0.04;
    private static final double MIN_POWER = 0.05;
    private static final double TX_TOLERANCE = 1.0;

    private boolean aligning = false;

    public AprilTagAlignHelper init(Limelight3A ll, MotorEx fl, MotorEx fr,
                                    MotorEx bl, MotorEx br, Telemetry tel) {
        this.limelight = ll;
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.telemetry = tel;
        return this;
    }

    @Override
    public void initialize() {
        limelight.pipelineSwitch(0);
        limelight.start();
        fr.reversed();
        br.reversed();
    }

    public void enableAlign() {
        aligning = true;
    }

    public void disableAlign() {
        aligning = false;
        stop();
    }

    @Override
    public void periodic() {
        if (!aligning) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            stop();
            return;
        }

        double tx = result.getTx();
        double rotationPower = kP * tx;

        if (Math.abs(rotationPower) < MIN_POWER && Math.abs(tx) > TX_TOLERANCE)
            rotationPower = Math.copySign(MIN_POWER, rotationPower);

        if (Math.abs(tx) <= TX_TOLERANCE) {
            stop();
            return;
        }

        rotate(rotationPower);
    }

    private void rotate(double power) {

        fl.set(power);
        bl.set(power);
        fr.set(-power);
        br.set(-power);
    }

    private void stop() {
        fl.set(0);
        bl.set(0);
        fr.set(0);
        br.set(0);
    }
}
