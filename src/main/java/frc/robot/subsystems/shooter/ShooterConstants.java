package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterConstants {
  public static final int feedId = 12;
  public static final int shootId = 11;

  public static final int laserCanId = 19;

  public static final boolean wheelInverted = false;

  public static final double wheelEncoderPositionFactor = 2 * Math.PI;
  public static final double wheelEncoderVelocityFactor = 2 * Math.PI / 60;

  public static final double maxVelocity = 5500 * 2 * Math.PI / 60;  // Rpm to radians to velocity


  public static final double shootKs = 0.0;
  public static final double shootKv = 12.0 / maxVelocity;
  public static final double shootKa = 0.0;

  public static final double shootKp = 0.0001;
  public static final double shootKd = 0.0;

  public static final DCMotor shootMotorGearbox = DCMotor.getNEO(1);
  public static final double shootMOI = 0.00351;

  public static final double detectedTheshold = 10.0;
}
