// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Map<RobotState, Trigger> stateRequests = new EnumMap<>(RobotState.class);
  private Map<RobotState, Trigger> stateTriggers = new EnumMap<>(RobotState.class);

  @AutoLogOutput(key = "RobotState/CurrentState")
  private RobotState state = RobotState.IDLE;

  @AutoLogOutput(key = "RobotState/PreviousState")
  private RobotState previousState = RobotState.IDLE;

  private double shootVelocityTarget = 0.0;
  private Rotation2d angleTarget = new Rotation2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(cameraName, robotToCamera));

        shooter = new Shooter(new ShooterIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(cameraName, robotToCamera, drive::getPose));
        shooter = new Shooter(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    stateRequests.put(RobotState.IDLE, controller.y());
    stateRequests.put(RobotState.INTAKE, controller.leftTrigger());
    stateRequests.put(RobotState.EJECT, controller.rightBumper());
    stateRequests.put(RobotState.READY, controller.povLeft());
    stateRequests.put(RobotState.AUTO_SCORE, controller.povUp());
    stateRequests.put(RobotState.MANUAL_SCORE, controller.povDown());
    stateRequests.put(RobotState.SCORE, controller.rightTrigger());
    stateRequests.put(RobotState.MANUAL, controller.leftBumper());

    for (RobotState state : RobotState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    stateTriggers
        .get(RobotState.IDLE)
        .and(stateRequests.get(RobotState.INTAKE))
        .onTrue(forceState(RobotState.INTAKE));

    stateTriggers
        .get(RobotState.IDLE)
        .or(stateTriggers.get(RobotState.INTAKE))
        .and(shooter::getDetected)
        .onTrue(forceState(RobotState.READY));

    stateTriggers
        .get(RobotState.INTAKE)
        .and(stateRequests.get(RobotState.IDLE))
        .onTrue(forceState(RobotState.IDLE));

    stateTriggers
        .get(RobotState.EJECT)
        .or(stateTriggers.get(RobotState.READY))
        .or(stateTriggers.get(RobotState.AUTO_SCORE))
        .or(stateTriggers.get(RobotState.MANUAL_SCORE))
        .and(() -> !shooter.getDetected())
        .onTrue(forceState(RobotState.IDLE));

    stateTriggers
        .get(RobotState.READY)
        .or(stateTriggers.get(RobotState.AUTO_SCORE))
        .or(stateTriggers.get(RobotState.MANUAL_SCORE))
        .and(stateRequests.get(RobotState.IDLE))
        .onTrue(forceState(RobotState.EJECT));

    stateTriggers
        .get(RobotState.READY)
        .or(stateTriggers.get(RobotState.MANUAL_SCORE))
        .and(stateRequests.get(RobotState.AUTO_SCORE))
        .onTrue(forceState(RobotState.AUTO_SCORE));

    stateTriggers
        .get(RobotState.READY)
        .or(stateTriggers.get(RobotState.AUTO_SCORE))
        .and(stateRequests.get(RobotState.MANUAL_SCORE))
        .onTrue(forceState(RobotState.MANUAL_SCORE));

    stateTriggers
        .get(RobotState.AUTO_SCORE)
        .and(stateRequests.get(RobotState.SCORE))
        // detect if parameters are correct
        .and(
            () -> {
              return true;
            })
        .onTrue(forceState(RobotState.SCORE));

    stateTriggers
        .get(RobotState.MANUAL_SCORE)
        .and(stateRequests.get(RobotState.SCORE))
        .onTrue(forceState(RobotState.SCORE));

    stateTriggers
        .get(RobotState.SCORE)
        .and(() -> !shooter.getDetected())
        .onTrue(Commands.sequence(Commands.waitSeconds(0.5), forceState(RobotState.IDLE)));

    stateTriggers
        .get(RobotState.MANUAL)
        .negate()
        .and(stateRequests.get(RobotState.MANUAL))
        .onTrue(forceState(RobotState.MANUAL));

    stateTriggers
        .get(RobotState.MANUAL)
        .and(stateRequests.get(RobotState.IDLE))
        .onTrue(forceState(RobotState.IDLE));

    stateTriggers
        .get(RobotState.MANUAL)
        .and(stateRequests.get(RobotState.READY))
        .onTrue(forceState(RobotState.READY));

    // State commands
    stateTriggers
        .get(RobotState.IDLE)
        .or(stateTriggers.get(RobotState.READY))
        .whileTrue(Commands.run(shooter::stop));

    stateTriggers.get(RobotState.INTAKE).whileTrue(Commands.run(shooter::intake));

    stateTriggers.get(RobotState.EJECT).whileTrue(Commands.run(shooter::eject));

    stateTriggers
        .get(RobotState.AUTO_SCORE)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> angleTarget));

    stateTriggers
        .get(RobotState.AUTO_SCORE)
        .whileTrue(Commands.run(() -> shooter.shoot(shootVelocityTarget)));

    stateTriggers
        .get(RobotState.AUTO_SCORE)
        .whileTrue(
            Commands.run(
                () -> {
                  double timeOfFlight = distanceToTarget() * 0.0;
                  Pose2d lookahead = drive.getLookahead(timeOfFlight);
                  Translation2d lookaheadDelta =
                      AllianceFlipUtil.apply(targetPosition).minus(lookahead.getTranslation());
                  double distance = lookaheadDelta.getNorm();
                  // INSERT REGRESSIONS HERE
                  angleTarget = lookaheadDelta.getAngle().plus(new Rotation2d(distance * 0.0));
                  shootVelocityTarget = distance * 1.0;
                }));

    stateTriggers
        .get(RobotState.MANUAL_SCORE)
        .and(controller.leftTrigger())
        .whileTrue(Commands.run(shooter::shoot));

    stateTriggers.get(RobotState.SCORE).whileTrue(Commands.run(shooter::feed));

    stateTriggers
        .get(RobotState.MANUAL)
        .and(controller.rightTrigger())
        .whileTrue(Commands.runEnd(shooter::feed, shooter::stopFeed));

    stateTriggers
        .get(RobotState.MANUAL)
        .and(controller.leftTrigger())
        .whileTrue(Commands.runEnd(shooter::shoot, shooter::stopShoot));

    stateTriggers
        .get(RobotState.MANUAL)
        .and(controller.rightBumper())
        .whileTrue(Commands.runEnd(shooter::intake, shooter::stop));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command forceState(RobotState nextState) {
    return Commands.runOnce(
        () -> {
          System.out.println("Changing state to " + nextState);
          previousState = state;
          state = nextState;
        });
  }

  @AutoLogOutput(key = "RobotState/DistanceToTarget")
  private double distanceToTarget() {
    return AllianceFlipUtil.apply(targetPosition).minus(drive.getPose().getTranslation()).getNorm();
  }

  @AutoLogOutput(key = "RobotState/AngleToTarget")
  private double angleToTarget() {
    return AllianceFlipUtil.apply(targetPosition)
        .minus(drive.getPose().getTranslation())
        .getAngle()
        .minus(drive.getRotation())
        .getRadians();
  }
}
