package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class flywheelTuning extends OpMode {
    private DcMotorEx flywheel1, flywheel2;
    public static double targetVelocity, velocity;
    public static double P,kV,kS;
    @Override
    public void init() {
        //TODO: Set motor name and direction
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("CurrentVel", velocity);
        double error = targetVelocity - velocity;
        double feedback = error * P;
        double feedforward = kV * targetVelocity + kS;
        double power = feedback + feedforward;
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }
}