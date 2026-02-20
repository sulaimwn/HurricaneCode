package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.servoMotor;
import org.firstinspires.ftc.teamcode.Flywheel;
@Config
@TeleOp(name = "TeleOp")

public class MecanumTeleop2025 extends OpMode {

    private DcMotorEx flywheelMotor, flywheelMotor2;
    private Servo gate;
    public static double kP = 0.02;       // proportional gain
    public static double kF = 0.00035;    // feedforward gain
    //public static double maxTargetTPS = 1500; // target speed in ticks/sec
    // LOW / HIGH target speeds
    public static double lowTargetTPS  = 800;
    public static double highTargetTPS = 1500;
    private static final double OUTPUT_MAX = 1.0;

    public static double openPos = 0.6, closePos = 0.76;

    private boolean lastAState = false;
    private boolean lastBState = false;


    //private boolean flywheelOn = false;  dont need i think


    private enum FlywheelState { OFF, LOW, HIGH };
    private FlywheelState flywheelState = FlywheelState.OFF;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");  // SECOND ADDED
        gate = hardwareMap.get(Servo.class, "gate");




//        FlywheelPID = new Flywheel(hardwareMap, telemetry);  Important code that we commented

        telemetry.update();
    }
    @Override


    //boolean currentPress = gamepad1.B;
    public void loop() {
        boolean currentA = gamepad2.a;
        boolean currentB = gamepad2.b;

        if (currentA && !lastAState) {

            if (flywheelState == FlywheelState.LOW) {
                flywheelState = FlywheelState.OFF;
            } else {
                flywheelState = FlywheelState.LOW;
            }
        }

        if (currentB && !lastBState) {

            if (flywheelState == FlywheelState.HIGH) {
                flywheelState = FlywheelState.OFF;
            } else {
                flywheelState = FlywheelState.HIGH;
            }
        }

        lastAState = currentA;
        lastBState = currentB;



        double targetTPS;
        if (flywheelState == FlywheelState.LOW) {
            targetTPS = lowTargetTPS;
        } else if (flywheelState == FlywheelState.HIGH) {
            targetTPS = highTargetTPS;
        } else {
            targetTPS = 0.0;
        }


        double measuredTPS = flywheelMotor.getVelocity(); // ticks/sec
        double power = kF * targetTPS + kP * (targetTPS - measuredTPS);
        power = Math.max(0, Math.min(OUTPUT_MAX, power)); // clamp to 0–1
        flywheelMotor.setPower(power);

        double measuredTPS2 = flywheelMotor2.getVelocity(); // ticks/sec
        double power2 = kF * targetTPS + kP * (targetTPS - measuredTPS2);
        power2 = Math.max(0, Math.min(OUTPUT_MAX, power2)); // clamp to 0–1
        flywheelMotor2.setPower(power2);


        telemetry.addData("Flywheel State", flywheelState);
        telemetry.addData("Target TPS", targetTPS);
        telemetry.addData("Low Speed Target", lowTargetTPS);
        telemetry.addData("High Speed Target", highTargetTPS);
        telemetry.addData("Motor 1 TPS", measuredTPS);
        telemetry.addData("Motor 1 Power", power);
        telemetry.addData("Motor 2 TPS", measuredTPS2);
        telemetry.addData("Motor 2 Power", power2);
        telemetry.addData("TPS Difference", Math.abs(measuredTPS - measuredTPS2));

        if (gamepad1.a) {
            gate.setPosition(openPos);
        } else {
            gate.setPosition(closePos);
        }


    }

    public void stop() {
        if (flywheelMotor != null) flywheelMotor.setPower(0.0);
        if (flywheelMotor2 != null) flywheelMotor2.setPower(0.0);
    }
}