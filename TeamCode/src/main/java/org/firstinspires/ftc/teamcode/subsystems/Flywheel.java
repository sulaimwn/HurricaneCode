//AKA shooter

package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }

    private final MotorEx motor = new MotorEx("flywheelMotor");
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public final Command on = new RunToVelocity(controller, 500.0).requires(this).named("FlywheelOn"); //this isnt in the tele op for some reason i just added this (5pm saturday nov 11)

    public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff");

    //public final Command lowPower = new RunToVelocity(controller, 500).requires(this).named("FlywheelOn");

    // tweak RPM in this command we want it to be at 2000
   // public final Command mediumPower = new RunToVelocity(controller, 1000).requires(this).named("FlywheelOn");


    //INTAKING COMMAND - Here its just off
    public final Command intaking = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelIntaking"); // I assume that by setting the same names they all operate at once on the same command

    //DRIVING COMMAND - Here its just off

    public final Command driving = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelDriving");


    //SHOOTING COMMAND 3 different versions
    public final Command shooting1500 = new RunToVelocity(controller, 1500.0).requires(this).named("shooting1500");
    public final Command shooting2000 = new RunToVelocity(controller, 2000.0).requires(this).named("shooting2000");
    public final Command shooting2500 = new RunToVelocity(controller, 2500.0).requires(this).named("shooting2500");
    public final Command flywheel1500RPM = new RunToVelocity(controller, 500.0).requires(this).named("Flywheel1500aRPM");
    public final Command flywheel2000RPM = new RunToVelocity(controller, 2000).requires(this).named("Flywheel2000RPM");
    public final Command flywheel2500RPM = new RunToVelocity(controller, 2500).requires(this).named("Flywheel2500RPM");
    public final Command highPower = new RunToVelocity(controller, 2000.0).requires(this).named("FlywheelOn");
    @Override
    // runs every loop
    public void periodic() {

        // see if the goal is measured in RPM or something else
        System.out.println("Flywheel velocity: " + motor.getState());
        motor.setPower(controller.calculate(motor.getState()));
    }
}