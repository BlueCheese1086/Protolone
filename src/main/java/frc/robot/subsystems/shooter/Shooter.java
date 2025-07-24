package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  LoggedNetworkNumber shootVelocity =
      new LoggedNetworkNumber("Tuning/Shooter/ShootVelocity", maxVelocity);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(shootKs, shootKv, shootKa);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void shoot(double velocityRadPerSec) {
    io.setShootVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));
  }

  public void shoot() {
    shoot(shootVelocity.get());
  }

  public void feed() {
    io.setFeedOpenLoop(12.0);
  }

  public void intake() {
    io.setShootOpenLoop(-3.0);
    io.setFeedOpenLoop(-3.0);
  }

  public void eject() {
    io.setShootOpenLoop(3.0);
    io.setFeedOpenLoop(3.0);
  }

  public void stop() {
    io.setShootOpenLoop(0.0);
    io.setFeedOpenLoop(0.0);
  }

  public void stopShoot() {
    io.setShootOpenLoop(0.0);
  }

  public void stopFeed() {
    io.setFeedOpenLoop(0.0);
  }

  public boolean getDetected() {
    return inputs.noteDetected;
  }
}
