package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class FlywheelPIDF extends OpMode {

    private DcMotorEx flywheelMotor;
    private PIDFController pidfController;
    private Telemetry telem;

    public static double p = 0.0;
    public static double i = 0.0;
    public static double kV = 0.0;
    public static double targetVelocity = 0.0;

    private boolean flywheelOn = false;
    private boolean lastA = false;

    @Override
    public void init() {
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        pidfController = new PIDFController(p, i, 0.0, 0.0);
    }

    @Override
    public void loop() {

        if (gamepad1.a && !lastA) flywheelOn = !flywheelOn;
        lastA = gamepad1.a;

        pidfController.setP(p);
        pidfController.setI(i);
        pidfController.setD(0.0);
        pidfController.setF(kV * targetVelocity);

        if (flywheelOn) {
            double power = pidfController.calculate(flywheelMotor.getVelocity(), targetVelocity);
            flywheelMotor.setPower(power);
        } else {
            flywheelMotor.setPower(0);
        }

        telem.addData("Flywheel ON?", flywheelOn);
        telem.addData("Current Velocity (Units/sec)", flywheelMotor.getVelocity());
        telem.addData("Target Velocity (Units/sec)", targetVelocity);
        telem.update();
    }
}