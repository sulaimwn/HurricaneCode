package org.firstinspires.ftc.teamcode.opModes.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Red_Auto", group = "Autonomous")
public class autonomous_Red extends LinearOpMode {

    // ===================== SHOOTER SUBSYSTEM =====================
    public static class Shooter {
        // --- Dashboard tunables ---
        public static double kP = 0.5;
        public static double kF = 0.00042;
        public static double OUTPUT_MAX = 1.0;
        public static double targetTPS = 1500.0;   // requested 1500 TPS

        public static double INTAKE_POWER   = 1.0;
        public static double TRANSFER_POWER = 1;

        private DcMotorEx flywheel;
        private DcMotorEx transfer;
        private DcMotorEx intake;

        private boolean flywheelEnabled = false;

        public Shooter(HardwareMap hardwareMap) {
            flywheel  = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
            transfer  = hardwareMap.get(DcMotorEx.class, "transferMotor");
            intake    = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            // (optional but recommended) configure modes/behaviors here

        }

        // ========= your TPS controller wrapped into a method =========
        public void updateFlywheel() {
            if (!flywheelEnabled) return;

            double measuredTPS = flywheel.getVelocity(); // ticks/sec
            double power = kF * targetTPS + kP * (targetTPS - measuredTPS);
            power = Math.max(0, Math.min(OUTPUT_MAX, power));
            flywheel.setPower(power);
        }

        private void enableFlywheel() {
            flywheelEnabled = true;
        }

        public void stopAll() {
            flywheelEnabled = false;
            flywheel.setPower(0);
            transfer.setPower(0);
            intake.setPower(0);
        }

        // ========= Actions =========

        // 1) Shooter action: spin flywheel with PIDF for a fixed time
        public Action spinFlywheelForSeconds(double durationSeconds) {
            return new SpinFlywheelAction(durationSeconds);
        }

        // 2) Transfer action: run transfer motor for a fixed time
        public Action runTransferForSeconds(double durationSeconds, double power) {
            return new TransferAction(durationSeconds, power);
        }

        // 3) Intake action: run intake motor for a fixed time
        public Action runIntakeForSeconds(double durationSeconds, double power) {
            return new IntakeAction(durationSeconds, power);
        }

        // ----- Inner Action classes -----

        private class SpinFlywheelAction implements Action {
            private final double duration;
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            SpinFlywheelAction(double durationSeconds) {
                this.duration = durationSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    enableFlywheel();
                }

                // update PIDF every loop
                updateFlywheel();

                packet.put("flywheelVel", flywheel.getVelocity());
                packet.put("targetTPS", targetTPS);

                if (timer.seconds() < duration) {
                    return true;    // keep running
                } else {
                    // just stop flywheel, leave intake/transfer alone
                    flywheelEnabled = false;
                    flywheel.setPower(0);
                    return false;
                }
            }
        }

        private class TransferAction implements Action {
            private final double duration;
            private final double power;
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            TransferAction(double durationSeconds, double power) {
                this.duration = durationSeconds;
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    transfer.setPower(power);
                }

                if (timer.seconds() < duration) {
                    return true;
                } else {
                    transfer.setPower(0);
                    return false;
                }
            }
        }

        private class IntakeAction implements Action {
            private final double duration;
            private final double power;
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            IntakeAction(double durationSeconds, double power) {
                this.duration = durationSeconds;
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    intake.setPower(power);
                }

                if (timer.seconds() < duration) {
                    return true;
                } else {
                    intake.setPower(0);
                    return false;
                }
            }
        }
    }

    // ===================== MAIN AUTO =====================
    @Override
    public void runOpMode() {
        Pose2d startPose      = new Pose2d(-51, 48, Math.toRadians(135));
        Pose2d scorePose      = new Pose2d(-25, 20, Math.toRadians(135));
        Pose2d firstLinePose  = new Pose2d(-12, 22, Math.toRadians(90));
        Pose2d secondLinePose = new Pose2d(12, 24, Math.toRadians(90));
        Pose2d thirdLinePose  = new Pose2d(-36, 43, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);

        int visionOutputPosition = 1; // fake vision result for now

        // Build the full auto path as one trajectory action
        TrajectoryActionBuilder shootingFirstBall = drive.actionBuilder(startPose)
                .strafeToLinearHeading(scorePose.position, scorePose.heading)
                .waitSeconds(4)



                .splineToLinearHeading(
                        new Pose2d(firstLinePose.position, firstLinePose.heading),
                        firstLinePose.heading)
                .lineToY(55)

                .splineToLinearHeading(new Pose2d(scorePose.position, scorePose.heading),
                        scorePose.heading)
                .splineToLinearHeading(
                        new Pose2d(scorePose.position, scorePose.heading),
                        scorePose.heading)
                //CHANGE
                .waitSeconds(5)
                .splineToLinearHeading(new Pose2d(secondLinePose.position, secondLinePose.heading), secondLinePose.heading)
                .lineToY(55)
                .lineToY(40)
                .splineToLinearHeading(
                        new Pose2d(scorePose.position, scorePose.heading),
                        scorePose.heading);


        Action firstBall = shootingFirstBall.build();



        // INIT LOOP (vision etc.)
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Example: spin up + feed while also running the path
        Actions.runBlocking(
                new ParallelAction(
                        shooter.spinFlywheelForSeconds(30),

                        firstBall,

                        new SequentialAction(

                                //First Ball Preloaded

                                new SleepAction(1.5),
                                shooter.runTransferForSeconds(0.5, autonomous_Red.Shooter.TRANSFER_POWER),
                                new SleepAction(1),
                                //Second Ball Preloaded
                                shooter.runIntakeForSeconds(0.5, autonomous_Red.Shooter.INTAKE_POWER),
                                shooter.runTransferForSeconds(1, autonomous_Red.Shooter.TRANSFER_POWER),
                                shooter.runIntakeForSeconds(0.5, autonomous_Red.Shooter.INTAKE_POWER),

                                //Thirdball Preloaded
                                shooter.runTransferForSeconds(0.5, autonomous_Red.Shooter.TRANSFER_POWER),


                                new SleepAction(2),

                                //CHANGE TIME
                                shooter.runIntakeForSeconds(5, autonomous_Red.Shooter.INTAKE_POWER),

                                //FirstBall FirstLIne
                                shooter.runTransferForSeconds(0.5, autonomous_Red.Shooter.TRANSFER_POWER),

                                new SleepAction(1),
                                //Second Ball FirstLIne
                                shooter.runIntakeForSeconds(0.5, autonomous_Red.Shooter.INTAKE_POWER),
                                shooter.runTransferForSeconds(1, autonomous_Red.Shooter.TRANSFER_POWER),

                                shooter.runIntakeForSeconds(0.25, autonomous_Red.Shooter.INTAKE_POWER),

                                //Thirdball FirstLIne
                                shooter.runTransferForSeconds(0.5, autonomous_Red.Shooter.TRANSFER_POWER),
                                new SleepAction(2),


                                //intake SecondLine

                                shooter.runIntakeForSeconds(8, autonomous_Red.Shooter.INTAKE_POWER),


                                //FirstBall Second Line

                                shooter.runTransferForSeconds(0.5, autonomous_Red.Shooter.TRANSFER_POWER),
                                new SleepAction(1),
                                //SecondBall Second Line
                                shooter.runIntakeForSeconds(0.5, autonomous_Red.Shooter.INTAKE_POWER),
                                shooter.runTransferForSeconds(1, autonomous_Red.Shooter.TRANSFER_POWER),
                                shooter.runIntakeForSeconds(0.25, autonomous_Red.Shooter.INTAKE_POWER),

                                //Thirdball Second Line
                                shooter.runTransferForSeconds(0.5, autonomous_Red.Shooter.TRANSFER_POWER)



                        )
                )
        );

        shooter.stopAll();
    }
}