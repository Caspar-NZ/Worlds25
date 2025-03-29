package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Calibration op mode for the REV Color Sensor.
 *
 * How to use:
 * 1. Place the target color (e.g., red) in front of the sensor.
 * 2. Hold gamepad1's right bumper to update calibration values.
 * 3. Telemetry displays current, minimum, and maximum HSV values.
 * 4. Adjust your conditions and samples, then use the telemetry data for calibration.
 */
@TeleOp(name = "ZZColourCalibration", group = "Calibration")
@Disabled
public class ZZColourCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get the REV Color Sensor from the hardware map.
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");

        // Initialize calibration values.
        float minH = 360, maxH = 0;
        float minS = 1,   maxS = 0;
        float minV = 1,   maxV = 0;

        telemetry.addData("Status", "Calibrating - hold gamepad1 RB to update calibration values");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read normalized color values from the sensor.
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert normalized (0.0-1.0) values to 0-255 for RGB conversion.
            int r = (int)(colors.red * 255);
            int g = (int)(colors.green * 255);
            int b = (int)(colors.blue * 255);

            // Convert the RGB values to HSV.
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);

            // Only update calibration when the right bumper is held.
            if (gamepad1.right_bumper) {
                if (hsv[0] < minH) minH = hsv[0];
                if (hsv[0] > maxH) maxH = hsv[0];
                if (hsv[1] < minS) minS = hsv[1];
                if (hsv[1] > maxS) maxS = hsv[1];
                if (hsv[2] < minV) minV = hsv[2];
                if (hsv[2] > maxV) maxV = hsv[2];
            }

            // Display telemetry.
            telemetry.addData("Calibration Mode", gamepad1.right_bumper ? "Updating" : "Paused");
            telemetry.addData("Current HSV", String.format("H: %.1f  S: %.2f  V: %.2f", hsv[0], hsv[1], hsv[2]));
            telemetry.addData("Min HSV",     String.format("H: %.1f  S: %.2f  V: %.2f", minH, minS, minV));
            telemetry.addData("Max HSV",     String.format("H: %.1f  S: %.2f  V: %.2f", maxH, maxS, maxV));
            telemetry.update();

            // A short sleep to make telemetry readable.
            sleep(100);
        }
    }
}
