package frc.robot;

public final class Constants
{
    // =====================================================
    // CAN ID
    // =====================================================

    public static final int TITAN_ID = 42;


    // =====================================================
    // DRIVE MOTORS
    // =====================================================

    public static final int M0 = 0; // Right Motor
    public static final int M1 = 1; // Back Motor
    public static final int M3 = 3; // Left Motor


    // =====================================================
    // DRIVE ENCODERS
    // =====================================================

    // Omni wheel radius in mm
    public static final double wheelRadius = 55.0;

    // Encoder pulses per revolution
    public static final double pulsePerRevolution = 1440.0;

    // Encoder-to-wheel gear ratio
    public static final double gearRatio = 1.0 / 1.0;

    // Encoder pulses for one complete wheel revolution
    public static final double wheelPulseRatio =
            pulsePerRevolution * gearRatio;

    // Distance travelled per encoder tick in mm
    public static final double WHEEL_DIST_PER_TICK =
            (Math.PI * 2.0 * wheelRadius)
            / wheelPulseRatio;
}