
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
//import frc.robot.commands.ZeroGyroCommand;
//import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DisplayLogCommand;
import frc.robot.commands.DisplayLogCommand.DISPLAYMODE;

public class Autonomous {
  DrivetrainSubsystem m_drivetrainSubsystem;
  // private final BalanceCommand balanceCommand;
  public static SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public Autonomous(DrivetrainSubsystem m_drivetrainSubsystem) {
    // this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    // balanceCommand = new BalanceCommand(m_drivetrainSubsystem);
    autonomousChooser.setDefaultOption("Case 1 left", getAutonomousCommandCase1());
    autonomousChooser.addOption("Case 2 left turn", getAutonomousCommandCase2());
    autonomousChooser.addOption("Case 3 Center Balance", getAutonomousCommandCase3());
    autonomousChooser.addOption("Case 4 Center Pickup Balance", getAutonomousCommandCase4());

    // Put the chooser on the dashboard
    SmartDashboard.putData("Autonomous Mode", autonomousChooser);
  }

  private Command getAutonomousCommandCase1() {
    Command command = new DisplayLogCommand("Case 1", DISPLAYMODE.INIT_TIME)
        .andThen(new WaitCommand(1.0))
        .andThen(new DisplayLogCommand("Case 1", DISPLAYMODE.ELASPED));
    return command;
  }

  private Command getAutonomousCommandCase2() {
    Command command = new DisplayLogCommand("Case 2", DISPLAYMODE.INIT_TIME)
        .andThen(new WaitCommand(2.0))
        .andThen(new DisplayLogCommand("Case 2", DISPLAYMODE.ELASPED));
    return command;
  }

  private Command getAutonomousCommandCase3() {
    Command command = new DisplayLogCommand("Case 3", DISPLAYMODE.INIT_TIME)
        .andThen(new WaitCommand(3.0))
        .andThen(new DisplayLogCommand("Case 3", DISPLAYMODE.ELASPED));
    return command;
  }

  private Command getAutonomousCommandCase4() {
    Command command = new DisplayLogCommand("Case 4", DISPLAYMODE.INIT_TIME)
        .andThen(new WaitCommand(4))
        .andThen(new DisplayLogCommand("Case 4", DISPLAYMODE.ELASPED));
    return command;
  }

  // autonomousChooser.setDefaultOption("Over and Balance",

  // AutonomousCommandFactory.getAutonomousSimpleLowCommand(m_drivetrainSubsystem,
  // m_armSubsystem, grabberSubsystem)
  // .andThen(AutonomousCommandFactory.getOverAndBalanceCommand(m_drivetrainSubsystem,
  // poseEstimator)));
  // // A chooser for autonomous commands
  // autonomousChooser.setDefaultOption("Middle Balance",
  // AutonomousCommandFactory.getAutonomousSimpleCommand(m_drivetrainSubsystem,
  // m_armSubsystem, grabberSubsystem)
  // .andThen(AutonomousCommandFactory.getSetPositionAndBalanceCommand(m_drivetrainSubsystem,
  // poseEstimator)));
  // autonomousChooser.addOption("Simple Case and Left out",
  // AutonomousCommandFactory.getAutonomousAcceleratedAndLeftOutCommand(m_drivetrainSubsystem,
  // m_armSubsystem,
  // grabberSubsystem));
  // autonomousChooser.addOption("Simple Case and Right out",
  // AutonomousCommandFactory.getAutonomousSimpleAndRightDeacceleratedOutCommand(m_drivetrainSubsystem,
  // m_armSubsystem,
  // grabberSubsystem));

  // // Add commands to the autonomous command chooser
  // autonomousChooser.addOption("Case 1 left",
  // getAutonomousCommandCase1(0).andThen(new
  // StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 2),
  // getFinalPoseForCase1(0))));

  // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
  // autonomousChooser.addOption("CaSe 1 right", getAutonomousCommandCase1(2)
  // .andThen(new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // getFinalPoseForCase1(2))));

  // autonomousChooser.addOption("Case 2 left", getAutonomousCommandCase2(0));
  // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
  // autonomousChooser.addOption("Case 2 red", getAutonomousCommandCase2Red());
  // autonomousChooser.addOption("Case 2 blue", getAutonomousCommandCase2Blue());
  // autonomousChooser.addOption("Case 2 right", getAutonomousCommandCase2(2));
  // autonomousChooser.addOption("Case 3", getAutonomousCommandCase3());

  public Command getAutonomousCommandCase2Red() {
    Command command = new DisplayLogCommand("Case 2 Red")

        // .andThen(new GrabberCommand(grabberSubsystem, false))
        // .andThen(new KeyPadStateCommand(1))
        // .andThen(getCommandFor(1))
        // .andThen(new GrabberCommand(grabberSubsystem, true))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // new Pose2d(new Translation2d(2.1, 5.24), new
        // Rotation2d(Math.toRadians(180)))))
        // .andThen(new ZeroShoulderCommand(m_armSubsystem))
        // .andThen(new ChangeTurboModeCommand())
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        .andThen(new WaitCommand(0.5));
    // .andThen(new ChangeNormalModeCommand())
    // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
    // .andThen(new WaitCommand(2.5))
    // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
    // // .andThen(new WaitCommand(2))
    // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
    // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
    command.setName("Case 2 red");
    return command;
  }

  // public Command getAutonomousCommandCase2Blue() {
  // Command command = new ZeroGyroCommand(m_drivetrainSubsystem,
  // balanceCommand, (180))
  // .andThen(new GrabberCommand(grabberSubsystem, false))
  // .andThen(new KeyPadStateCommand(1))
  // .andThen(getCommandFor(1))
  // .andThen(new GrabberCommand(grabberSubsystem, true))
  // .andThen(new WaitCommand(0.5))
  // .andThen(new ZeroExtenderCommand(m_armSubsystem))
  // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
  // new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
  // new Rotation2d(Math.toRadians(180)))))
  // .andThen(new ZeroShoulderCommand(m_armSubsystem))
  // .andThen(new ChangeTurboModeCommand())
  // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
  // .andThen(new WaitCommand(0.5))
  // .andThen(new ChangeNormalModeCommand())
  // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
  // .andThen(new WaitCommand(2.5))
  // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
  // // .andThen(new WaitCommand(2))
  // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
  // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
  // command.setName("Case 2 blue");
  // return command;
  // }

  // public Command getAutonomousCommandCase3() {
  // KeyPadPositionSupplier.state = 0;
  // Command command = new Command() {
  // @Override
  // public void initialize() {
  // KeyPadPositionSupplier.state = 0;
  // }

  // @Override
  // public boolean isFinished() {
  // return true;
  // }

  // }.andThen(getCommandFor(0)
  // .andThen(new GrabberCommand(grabberSubsystem, true))
  // // .andThen(new WaitCommand(1))
  // .andThen(new ZeroExtenderCommand(m_armSubsystem))
  // // .andThen(
  // // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
  // // new Pose2d(5.75,
  // // 7.4,
  // // new Rotation2d(Math.toRadians(180)))))
  // .andThen(
  // new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // new Pose2d(4.4,
  // 5.3,
  // new Rotation2d(Math.toRadians(0)))))
  // .andThen(
  // new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // new Pose2d(6.4,
  // 5.3,
  // new Rotation2d(Math.toRadians(0)))))
  // .andThen(new ShoulderCommand(m_armSubsystem, 40352))
  // .andThen(new ExtenderCommand(m_armSubsystem, 183023 * 16 / 36))
  // .andThen(new GrabberCommand(grabberSubsystem, false))
  // .andThen(new GrabberCommand(grabberSubsystem, false))
  // .andThen(new WaitCommand(2))
  // .andThen(new ZeroExtenderCommand(m_armSubsystem))
  // .andThen(Commands.parallel(
  // new ShoulderCommand(m_armSubsystem, 190432),
  // new StraightPathCommand(m_drivetrainSubsystem,
  // getPoseEstimatorForTarget(poseEstimator, 0),
  // new Pose2d(3.2, 5.3,
  // new Rotation2d(Math.toRadians(0))))))
  // .andThen(getCommandFor(4))
  // .andThen(new GrabberCommand(grabberSubsystem, true)));
  // command.setName("case 3");
  // return command;
  // }

}