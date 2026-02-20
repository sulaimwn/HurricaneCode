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
    public static double maxTargetTPS = 1500; // target speed in ticks/sec
    private static final double OUTPUT_MAX = 1.0;

    public static double openPos = 0.6, closePos = 0.76;

    private boolean lastButtonState = false;



    private boolean flywheelOn = false;
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
        boolean buttonPressed = gamepad2.a;
        if (buttonPressed && !lastButtonState) flywheelOn = !flywheelOn;
        lastButtonState = buttonPressed;
        double targetTPS = flywheelOn ? maxTargetTPS : 0.0;

        // Use motor.getVelocity() for measured speed
        double measuredTPS = flywheelMotor.getVelocity(); // ticks/sec
        double power = kF * targetTPS + kP * (targetTPS - measuredTPS);
        power = Math.max(0, Math.min(OUTPUT_MAX, power));
        flywheelMotor.setPower(power);

        //MOTOR 2 PID
        double measuredTPS2 = flywheelMotor2.getVelocity(); // ticks/sec
        double power2 = kF * targetTPS + kP * (targetTPS - measuredTPS2);
        power2 = Math.max(0, Math.min(OUTPUT_MAX, power2));
        flywheelMotor2.setPower(power2);

        // Telemetry
        telemetry.addData("Target TPS", targetTPS);
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