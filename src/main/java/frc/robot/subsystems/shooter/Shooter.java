package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.maxVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void shoot(double velocityRadPerSec) {
    io.setShootVelocity(velocityRadPerSec);
  }

  public void shoot() {
    shoot(maxVelocity);
  }

  public void feed() {
    io.setFeedOpenLoop(12.0);
  }

  public void intake() {
    io.setShootOpenLoop(-3.0);
    io.setFeedOpenLoop(-3.0);
  }

  public void stop() {
    io.setShootOpenLoop(0.0);
    io.setFeedOpenLoop(0.0);
  }

  public boolean getDetected() {
    return inputs.noteDetected;
  }
}
