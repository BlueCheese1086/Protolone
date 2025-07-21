package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean feedConnected = false;
    public double feedAppliedVolts = 0.0;
    public double feedCurrentAmps = 0.0;

    public boolean shootConnected = false;
    public double shootVelocityRadPerSec = 0.0;
    public double shootAppliedVolts = 0.0;
    public double shootCurrentAmps = 0.0;

    public boolean laserCanConnected;
    public boolean noteDetected;
    public double laserCanDistance;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setFeedOpenLoop(double output) {}

  public default void setShootOpenLoop(double output) {}

  public default void setShootVelocity(double velocityRadPerSec) {}
}
