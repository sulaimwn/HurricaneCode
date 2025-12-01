package org.firstinspires.ftc.teamcode.opmodes;


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
import org.firstinspires.ftc.teamcode.classes.AprilTagAlignHelper;
import org.firstinspires.ftc.teamcode.classes.Flywheel;
import org.firstinspires.ftc.teamcode.classes.transferGate;
@Config
@TeleOp(name = "TeleOp")

public class MecanumTeleop2025 extends OpMode {

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intakeMotor, transferMotor, flywheelMotor;
    public static double kP = 0.02;       // proportional gain
    public static double kF = 0.00035;    // feedforward gain
    public static double maxTargetTPS = 1500; // target speed in ticks/sec
    private static final double OUTPUT_MAX = 1.0;
    private Servo gate;

    AprilTagAlignHelper aprilTagAlign;



    String gateLabel;

    private boolean lastButtonState = false;
    public static double openPos = 0.6, closePos = 0.76;
    double intakePower = 0.0, transferPower = 0.0, rx;

    private boolean lastGateState = false;
    private boolean servoToggled = false;
    private boolean flywheelOn = false;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        gate = hardwareMap.get(Servo.class, "gate");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // create objects
        aprilTagAlign = new AprilTagAlignHelper(hardwareMap,telemetry);

//        FlywheelPID = new Flywheel(hardwareMap, telemetry);

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

        // Telemetry
        telemetry.addData("Target TPS", targetTPS);
        telemetry.addData("Measured TPS", measuredTPS);
        telemetry.addData("Power", power);

//        if (servoToggled){
//          gateLabel = "close";}
//        else{
//            gateLabel = "open";;
//        }

        //boolean buttonPressed = gamepad2.a;
        //if (buttonPressed && !lastButtonState) flywheelOn = !flywheelOn;
        //lastButtonState = buttonPressed;
        //double targetTPS = flywheelOn ? TARGET_RPM : 0.0;
        if (gamepad2.left_bumper){
            intakeMotor.setPower(-1);
        } else{
            intakeMotor.setPower(0);
        }

        if (gamepad2.right_bumper){
            transferMotor.setPower(-1);
        }
        else{
            transferMotor.setPower(0);

        }

        if (gamepad2.left_bumper){
            intakePower = -1;}

        else if (gamepad2.right_trigger > 0.05) {intakePower = gamepad2.right_trigger; // reverse with variable speed
        intakeMotor.setPower(intakePower);}

        // Right bumper = transfer forward; Left trigger = reverse transfer
        if (gamepad2.right_bumper){
            transferPower = -0.5;}
        else if (gamepad2.left_trigger > 0.05)
        {transferPower = gamepad2.left_trigger; // reverse with variable speed
        transferMotor.setPower(transferPower);}

        boolean currentButtonState = gamepad2.b;  // change button as needed

        // Toggle only when the button is pressed down (edge detection)
        if (currentButtonState && !lastGateState) {
            servoToggled = !servoToggled; // flips between true and false
        }

        // Update servo position
        if (servoToggled) {
            gate.setPosition(openPos);
        } else {
            gate.setPosition(closePos);
        }

        // Remember current button state
        lastGateState = currentButtonState;

        // if gamepad1.a is held returns true, if not it returns false

        if (gamepad1.a) {
            rx = aprilTagAlign.getRotationCorrection();
        } else {
            rx = gamepad1.right_stick_x;
        }
        // hold x to activate flywheel
        //FlywheelPID.update(gamepad2.x);

        // click B to open gate, then click B to close gate. work in progress


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);




    }

    public void stop() {
        if (flywheelMotor != null) flywheelMotor.setPower(0.0);
    }
}
