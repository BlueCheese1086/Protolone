package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.shootKd;
import static frc.robot.subsystems.shooter.ShooterConstants.shootKp;
import static frc.robot.subsystems.shooter.ShooterConstants.shootMOI;
import static frc.robot.subsystems.shooter.ShooterConstants.shootMotorGearbox;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim shootMotor;
  private final PIDController shootController;

  private final LoggedNetworkBoolean noteDetected =
      new LoggedNetworkBoolean("Tuning/Shooter/SimNotedDetected", false);
  private boolean feedRunningForward = false;
  private boolean feedRunningBackward = false;

  public ShooterIOSim() {
    shootMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(shootMotorGearbox, shootMOI, 1.0),
            shootMotorGearbox);
    shootController = new PIDController(shootKp, 0.0, shootKd);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.feedConnected = true;
    inputs.shootConnected = true;

    inputs.shootVelocityRadPerSec = shootMotor.getAngularVelocityRadPerSec();
    inputs.shootPositionRad = shootMotor.getAngularPositionRad();

    inputs.feedAppliedVolts = feedRunningForward ? 12.0 : (feedRunningBackward ? -12.0 : 0.0);
    inputs.shootAppliedVolts = shootMotor.getInputVoltage();

    inputs.feedCurrentAmps = Math.abs(inputs.feedAppliedVolts);
    inputs.shootCurrentAmps = shootMotor.getCurrentDrawAmps();

    inputs.noteDetected = noteDetected.get();
  }

  public void setFeedOpenLoop(double output) {
    feedRunningForward = output > 0.0;
    feedRunningBackward = output < 0.0;
  }

  public void setShootOpenLoop(double output) {
    shootMotor.setInputVoltage(output);
  }

  public void setShootVelocity(double velocityRadPerSec) {
    setShootVelocity(velocityRadPerSec, 0.0);
  }

  public void setShootVelocity(double velocityRadPerSec, double feedforward) {
    // Multiply by 12.0 to match spark max pid
    shootMotor.setInputVoltage(
        MathUtil.clamp(
            shootController.calculate(shootMotor.getAngularVelocityRadPerSec(), velocityRadPerSec)
                    * 12.0
                + feedforward,
            -12.0,
            12.0));
  }
}
