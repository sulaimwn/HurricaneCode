package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class servoMotor {
    private Servo ServoGate;
    private Telemetry telemetry;

    public static double openPos = 0.55, closePos = 0.9;

    // constructor
    public servoMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        ServoGate = hardwareMap.get(Servo.class, ("gate"));
    }
    public void OpenGate (boolean GateToggle) {

        if (GateToggle) {
            ServoGate.setPosition(openPos);
        }
    }

    public void CloseGate (boolean GateToggle) {

        if (!GateToggle) {

            ServoGate.setPosition(closePos);
        }

    }

}