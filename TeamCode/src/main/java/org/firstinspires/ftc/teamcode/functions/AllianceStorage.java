package org.firstinspires.ftc.teamcode.functions;

import org.firstinspires.ftc.teamcode.autos.SixSpecs;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */

public class AllianceStorage {
    public enum AllianceColor {
        RED,
        BLUE
    }
}
