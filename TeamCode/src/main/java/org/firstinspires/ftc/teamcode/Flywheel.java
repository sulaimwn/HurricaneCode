package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Flywheel {
    private DcMotorEx flywheelMotor;

    private DcMotorEx flywheelMotor2;

    private Telemetry telemetry;

    // PID coefficients
    public static double kP = 0.0012;
    public static double kI = 0.0;
    public static double kD = 0.0002;

    // Feedforward coefficient (tune this)
    public static double kF = 0.00035;

    public static double TARGET_RPM = 2500.0;
    public double integralSum = 0.0;
    public double lastError = 0.0;

    //more for second
    private double integralSum2 = 0.0;
    private double lastError2 = 0.0;

    public double velocity;
    public double velocity2;


    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        //second  make sure u CHANGE THESE NAMES to match your Rev Hub config
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

    }

    public void update(boolean isRunning) {
        if (!isRunning) {
            flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
            integralSum = 0;
            lastError = 0;
            //more for second
            integralSum2 = 0;
            lastError2 = 0;
            telemetry.addData("Flywheel", "Off");
            return;
        }

        velocity = flywheelMotor.getVelocity() * 60.0 / 28.0; // ticks/sec → RPM (adjust 28 for your motor’s ticks/rev)
        double error = TARGET_RPM - velocity;

        // PID
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double pidOutput = kP * error + kI * integralSum + kD * derivative;

        // Feedforward
        double feedforward = kF * TARGET_RPM;

        double output = pidOutput + feedforward;
        output = Math.max(0, Math.min(1, output)); // Clamp between 0–1

        flywheelMotor.setPower(output);

        // MORE FOR SECOND //////////////////
        velocity2 = flywheelMotor2.getVelocity() * 60.0 / 28.0; // ticks/sec → RPM
        double error2 = TARGET_RPM - velocity2;

        integralSum2 += error2;
        double derivative2 = error2 - lastError2;
        lastError2 = error2;

        double pidOutput2 = kP * error2 + kI * integralSum2 + kD * derivative2;
        double feedforward2 = kF * TARGET_RPM;
        double output2 = pidOutput2 + feedforward2;
        output2 = Math.max(0, Math.min(1, output2)); // Clamp 0–1

        flywheelMotor2.setPower(output2);
        /// /////////////////////////////

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Motor 1 RPM", velocity);
        telemetry.addData("Motor 1 Power", output);
        telemetry.addData("Motor 2 RPM", velocity2);
        telemetry.addData("Motor 2 Power", output2);
        telemetry.addData("Error", error);
        telemetry.addData("Error", error2);
        telemetry.addData("RPM Difference", Math.abs(velocity - velocity2));



    }
}