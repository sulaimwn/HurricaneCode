package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Teleop")
public class teleop extends OpMode {

    DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, rightmotor, leftmotor, intake;

    Servo hood, gate;
//    public TurretMechanism turret = new TurretMechanism();

    public static double targetVelocity, velocity;
    boolean closeOn = false;
    boolean farOn = false;

    boolean lastA = false;
    boolean lastX = false;

    public static double farVel = 2000;
    public static double closeVel = 1500;

    public static double hoodClosePos = 0.30;
    public static double hoodFarPos   = 0.55;

    boolean gateOpen = false;
    boolean lastY = false;

    public static double gateClosedPos = 0.20;
    public static double gateOpenPos   = 0.55;

    public static double P = 0 ,kV = 0,kS = 0;
    private Limelight3A limelight;

    @Override
    public void init() {
//        turret.init(hardwareMap, telemetry);
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheel
        rightmotor = hardwareMap.get(DcMotorEx.class, "rightmotor");
        leftmotor = hardwareMap.get(DcMotorEx.class, "leftmotor");

        // Typical mirrored flywheel setup
        rightmotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Drivetrain brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gate.setPosition(gateClosedPos);
    }

    @Override
    public void start() {
//        turret.resetTimer();
    }

    @Override
    public void loop() {
        //Meeccaum drivtrain
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        driveMecanum(y, x, rx);

        if (gamepad2.a && !lastA) {
            closeOn = !closeOn;
            if (closeOn) farOn = false;   // only one mode at a time
        }
        lastA = gamepad2.a;

// X → toggle FAR on/off
        if (gamepad2.x && !lastX) {
            farOn = !farOn;
            if (farOn) closeOn = false;   // only one mode at a time
        }
        lastX = gamepad2.x;
//Toggle gate servo
        if (gamepad2.y && !lastY) {
            gateOpen = !gateOpen;
        }
        lastY = gamepad2.y;

// Apply position
        gate.setPosition(gateOpen ? gateOpenPos : gateClosedPos);


        if (farOn) {
            targetVelocity = farVel;
            hood.setPosition(hoodFarPos);
            updateFlywheel(targetVelocity);

        } else if (closeOn) {
            targetVelocity = closeVel;
            hood.setPosition(hoodClosePos);
            updateFlywheel(targetVelocity);

        } else {
            // OFF = hard off
            rightmotor.setPower(0);
            leftmotor.setPower(0);
        }



                if (gamepad2.right_trigger > 0.05) {
                    intake.setPower(0.8);          // intake in, proportional
                } else if (gamepad2.left_trigger > 0.05) {
                    intake.setPower(-0.8);         // reverse out, proportional
                } else {
                    intake.setPower(0);           // off
                }


//        turret.update(true);
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("MeasuredVel", velocity);

        telemetry.addData("Mode", farOn ? "FAR" : closeOn ? "CLOSE" : "OFF");
        telemetry.update();
    }
    private void driveMecanum(double y, double x, double rx) {

        // Counteract imperfect strafing
        x *= 1.1;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    private void initHardware() {

        // Drivetrain
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheel
        rightmotor = hardwareMap.get(DcMotorEx.class, "rightmotor");
        leftmotor = hardwareMap.get(DcMotorEx.class, "leftmotor");

        // Typical mirrored flywheel setup
        leftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Drivetrain brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    private void updateFlywheel(double targetVelocity) {

        // Single encoder
        velocity = Math.abs(rightmotor.getVelocity()); // or use -rightFlywheel.getVelocity()

        double error = targetVelocity - velocity;
        double feedback = error * P;

        double feedforward = (targetVelocity == 0) ? 0 : (kV * targetVelocity + kS);

        double power = feedback + feedforward;

        rightmotor.setPower(power);
        leftmotor.setPower(power);

        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("MeasuredVel", velocity);
        telemetry.addData("Power", power);

    }
}