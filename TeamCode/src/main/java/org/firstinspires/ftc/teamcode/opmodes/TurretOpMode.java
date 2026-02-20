package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.classes.TurretMechanism;
@Config
@TeleOp(name="Tune Turret PD")
public class TurretOpMode extends OpMode {

    private TurretMechanism turret = new TurretMechanism();

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
    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {


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

        turret.update(true);

        // Telemetry
        telemetry.addData("Status", "Tuning Mode");
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addLine("-------------------");
        telemetry.addData("P Gain", "%.5f", turret.getKP());
        telemetry.addData("D Gain", "%.5f", turret.getKD());
        telemetry.update();
    }
}