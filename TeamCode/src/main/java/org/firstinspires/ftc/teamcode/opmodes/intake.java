package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "IntakeServoTest")
public class intake extends OpMode {

    private DcMotorEx intake;
    private Servo gate;

    public static double posA = 0.28;
    public static double posB = 0.50;

    boolean servoState = false;
    boolean lastX = false;

    @Override
    public void init() {
        gate = hardwareMap.get(Servo.class, "gate");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // Intake
        intake.setPower(gamepad1.right_trigger);

        // Toggle servo on X press
        if (gamepad1.x && !lastX) {
            servoState = !servoState;
        }
        lastX = gamepad1.x;

        // Apply correct position
        if (servoState) {
            gate.setPosition(posB);
        } else {
            gate.setPosition(posA);
        }

        telemetry.addData("ServoState", servoState ? "B" : "A");
        telemetry.addData("ServoPos", gate.getPosition());
        telemetry.update();
    }
}