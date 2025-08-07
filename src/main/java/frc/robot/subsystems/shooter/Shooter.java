package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  LoggedNetworkNumber shootVelocity = new LoggedNetworkNumber("Tuning/Shooter/ShootVelocity", maxVelocity);

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
    SlewRateLimiter ramp = new SlewRateLimiter(100);
    double rampedTarget = ramp.calculate(velocityRadPerSec);
    io.setShootVelocity(rampedTarget, feedforward.calculate(velocityRadPerSec));
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

  public void runVoltageShooter(Voltage voltage) {
    io.setShootOpenLoop(voltage.in(Volts));
  }

  public void stopFeed() {
    io.setFeedOpenLoop(0.0);
  }

  public boolean getDetected() {
    return inputs.noteDetected;
  }

  public double getShootVelocity() {
    return shootVelocity.get();
  }

  public Voltage getShooterVoltage() {
    return Voltage.ofBaseUnits(inputs.shootAppliedVolts, Volts);
  }

  public AngularVelocity getShooterVelocityEncoder() {
    return AngularVelocity.ofBaseUnits(inputs.shootVelocityRadPerSec, RadiansPerSecond);
  }

  public Angle getShooterPosition() {
    return Angle.ofBaseUnits(inputs.shootPositionRad, Radians);
  }

  public Command getShooterSysIdQuasistatic(Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> runVoltageShooter(volts),
            log -> {
              log.motor("shooter")
                  .voltage(getShooterVoltage())
                  .angularVelocity(getShooterVelocityEncoder())
                  .angularPosition(getShooterPosition());
            },
            this))
        .quasistatic(direction);
  }

  public Command getShooterSysIdDynamic(Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> runVoltageShooter(volts),
            log -> {
              log.motor("shooter")
              .voltage(getShooterVoltage())
              .angularVelocity(getShooterVelocityEncoder())
              .angularPosition(getShooterPosition());
            },
            this))
        .dynamic(direction);
  }
}
