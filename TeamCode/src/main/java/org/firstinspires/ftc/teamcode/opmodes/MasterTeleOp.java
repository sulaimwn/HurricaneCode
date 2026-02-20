package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.classes.FinalTurretClass;
import org.firstinspires.ftc.teamcode.classes.TurretMechanism;
@Config

@TeleOp(name="MasterTeleOp")
public class MasterTeleOp extends OpMode {
    

    DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, flywheel1, flywheel2, intake;
    private FinalTurretClass turret = new FinalTurretClass();
    public static double targetVelocity, velocity;

    private IMU imu;
    public static double P,kV,kS;
    private Limelight3A limelight;
    boolean closeOn = false, farOn = false;
    boolean lastA = false, lastB = false;
    double farVel = 2000, closeVel = 1500;
    @Override
    public void init() {
        turret.init(hardwareMap, telemetry);
        initHardware();

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));


    }


    @Override
    public void start() {
        turret.resetTimer();
    }


    @Override
    public void loop() {


        double robotTurnRateDegPerSec = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //Meeccaum drivtrain
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        driveMecanum(y, x, rx);


        if (gamepad1.a && !lastA) {
            closeOn = !closeOn;
            if (closeOn) farOn = false;  // only one mode at a time
        }
        lastA = gamepad1.a;


        // B toggles FAR mode
        if (gamepad1.b && !lastB) {
            farOn = !farOn;
            if (farOn) closeOn = false;  // only one mode at a time
        }
        lastB = gamepad1.b;


        // Decide target based on toggles
        if (farOn) {
            targetVelocity = farVel;
            updateFlywheel(targetVelocity);
        } else if (closeOn) {
            targetVelocity = closeVel;
            updateFlywheel(targetVelocity);
        } else {
            // OFF = hard off
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }


        if (gamepad1.right_trigger > 0.05) {
            intake.setPower(0.8);
        } else if (gamepad1.left_trigger > 0.05) {
            intake.setPower(-0.8);
        } else {
            intake.setPower(0);
        }

        //call the turret class and update it : ) ) )
        turret.update(true, robotTurnRateDegPerSec);


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
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");


        // Typical mirrored flywheel setup
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);


//        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        intake = hardwareMap.get(DcMotorEx.class, "intake");


        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Drivetrain brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    private void updateFlywheel(double targetVel) {
        // Single encoder
        velocity = flywheel1.getVelocity();

        double error = targetVel - velocity;
        double feedback = error * P;


        double feedforward = 0;
        if (targetVel > 0) {
            feedforward = kV * targetVel + kS;
        }

        double power = feedback + feedforward;

        flywheel1.setPower(power);
        flywheel2.setPower(power);


    }


}

