package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public static final int feedId = 12;
  public static final int shootId = 11;

  public static final int laserCanId = 19;

  public static final boolean wheelInverted = false;

  public static final double wheelEncoderPositionFactor = 2 * Math.PI;
  public static final double wheelEncoderVelocityFactor = 2 * Math.PI / 60;

  public static final double maxVelocity = 5500 * 2 * Math.PI;
  public static final double shootKv = 1.0 / maxVelocity;

  public static final double detectedTheshold = 10.0;
}
