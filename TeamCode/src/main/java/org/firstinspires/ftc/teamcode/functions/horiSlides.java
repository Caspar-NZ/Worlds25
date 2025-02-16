package org.firstinspires.ftc.teamcode.functions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class horiSlides {

    // Constants for PID control and range limits
    public static final int MAX_POSITION = 865;
    public static final int MIN_POSITION = 0;
    private static final double DEADZONE = 5; // Small threshold around target position
    private static final double MAX_POWER = 1; // Max power to prevent hitting stops

    // Define motors and PID controller
    private DcMotor leftHori;
    private DcMotor rightHori;
    private double targetPosition = 0;

    // PID Coefficients for controlling position
    private BasicPID pidController;
    private double SpeedControl = 1.0;

    // Constructor for initialising motors and PID controller
    public horiSlides(HardwareMap hardwareMap) {
        leftHori = hardwareMap.get(DcMotor.class, "leftHori");
        rightHori = hardwareMap.get(DcMotor.class, "rightHori");

        // Reverse direction for left motor
        rightHori.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders and set run mode
        leftHori.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHori.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHori.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHori.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up PID coefficients (adjust these based on tuning requirements)
        double kP = 0.02;  // Increased proportional gain
        double kI = 0.001; // Small integral gain to counteract gravity
        double kD = 0.001; // Derivative gain for smoothing
        PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
        pidController = new BasicPID(pidCoefficients);
    }

    // Set target position for the slides
    public void setPosition(double position, double Speed) {
        targetPosition = Math.max(MIN_POSITION, Math.min(position, MAX_POSITION));
        SpeedControl = Speed;
    }

    // Update method to move motors towards the target position
    public void update() {
        double currentPosLeft = leftHori.getCurrentPosition();
        double currentPosRight = rightHori.getCurrentPosition();
        double averagePosition = (currentPosLeft + currentPosRight) / 2.0;

        // Check if within deadzone around target to avoid oscillation
        if (Math.abs(averagePosition - targetPosition) < DEADZONE) {
            leftHori.setPower(0);
            rightHori.setPower(0);
            return;
        }

        // Calculate the power needed to reach target position
        double powerLeft = pidController.calculate(currentPosLeft, targetPosition);
        double powerRight = pidController.calculate(currentPosRight, targetPosition);

        // Clamp power to prevent overshooting
        powerLeft = Math.max(-MAX_POWER, Math.min(powerLeft, MAX_POWER));
        powerRight = Math.max(-MAX_POWER, Math.min(powerRight, MAX_POWER));

        powerLeft = powerLeft*SpeedControl;
        powerRight = powerRight*SpeedControl;

        // Set motor powers
        leftHori.setPower(-powerLeft);
        rightHori.setPower(-powerRight);
    }

    // Get current position of the slides (average of both motors for stability)
    public double getCurrentPosition() {
        return (leftHori.getCurrentPosition() + rightHori.getCurrentPosition()) / 2.0;
    }
}
