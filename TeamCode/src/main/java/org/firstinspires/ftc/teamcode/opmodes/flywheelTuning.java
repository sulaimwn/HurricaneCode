package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class flywheelTuning extends OpMode {
    private DcMotorEx rightFlywheel, leftFlywheel;
    public static double targetVelocity, velocity;
    public static double P,kV,kS;
    @Override
    public void init() {
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("CurrentVel", velocity);
        double error = targetVelocity - velocity;
        double feedback = error * P;
        double feedforward = kV * targetVelocity + kS;
        double power = feedback + feedforward;
        rightFlywheel.setPower(power);
        leftFlywheel.setPower(power);
    }
}