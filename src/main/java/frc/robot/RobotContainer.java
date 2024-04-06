              // ABDULLAH ABİM SAĞOLSUN\\
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorINSubsystem;
import frc.robot.subsystems.ElevatorOutSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer{

  public static final AngleSubsystem angleSub = new AngleSubsystem();
  public static final ClimberSubsystem climberSub = new ClimberSubsystem();
  public static final ElevatorINSubsystem elevatorINSub = new ElevatorINSubsystem();
  public static final ElevatorOutSubsystem elevatoroOutSub = new ElevatorOutSubsystem();
  public static final IntakeSubsystem intakeSub = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSub = new ShooterSubsystem();
  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));

  public static final CommandPS4Controller driver1 = new CommandPS4Controller(0);
  public static final CommandJoystick driver2 = new CommandJoystick(1);

  public RobotContainer(){
    configurePathPlanner();
    configureBindings();
    PortForwarder.add(5800, "photonvision.local", 5800);
    Command driveFieldOrientedAnglularVelocityPS5 = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driver1.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driver1.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driver1.getRawAxis(2));
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(() -> MathUtil.applyDeadband(driver1.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),() -> MathUtil.applyDeadband(driver1.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),() -> -driver1.getRawAxis(2));
    drivebase.setDefaultCommand( !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocityPS5 : driveFieldOrientedDirectAngleSim );
  }

  private void configureBindings() {
    //Gyro Reset
    driver1.button(1).whileTrue((new InstantCommand(drivebase::zeroGyro)));

    //Amfi Atma Pozisyonu
    driver2.button(1).onTrue(angleSub.SetAngle(-17));
    driver2.button(1).onTrue(elevatorINSub.SetElevator(-43));

    //Yerden Alma Pozisyonu
    driver2.button(2).onTrue(angleSub.SetAngle(-71.5));
    driver2.button(2).onTrue(elevatorINSub.SetElevator(-42));

    //Taşıma Pozisyonu
    driver2.button(3).onTrue(angleSub.SetAngle(-48.5));
    driver2.button(3).onTrue(elevatorINSub.SetElevator(-10));

    //Opsiyonel Pozisyonu
    driver2.button(4).onTrue(angleSub.SetAngle(-30));
    driver2.button(4).onTrue(elevatorINSub.SetElevator(-10));

    //Amfi Atış Hızı
    driver2.button(6).onTrue(intakeSub.AmpNote(-0.55));
    driver2.button(6).onTrue(shooterSub.shootNote(-0.3));
    driver2.button(6).onFalse(intakeSub.stopTeleopIntake());
    driver2.button(6).onFalse(shooterSub.stopShooter());

    //İntake
    driver2.button(7).onTrue(intakeSub.getNote(0.7));
    driver2.button(7).onFalse(intakeSub.stopTeleopIntake());

    //Sıfır
    driver2.button(5).onTrue(angleSub.SetAngle(0));
    driver2.button(5).onTrue(elevatorINSub.SetElevator(0));

    //Hoparlör Atış
    driver2.button(8).onTrue(intakeSub.waitAndShoot(0.75));
    driver2.button(8).onTrue(shooterSub.shootNote(0.9));
    driver2.button(8).onFalse(intakeSub.stopTeleopIntake());
    driver2.button(8).onFalse(shooterSub.stopShooter());

    //Tırmanma
    driver2.povUp().onTrue(climberSub.climbUp());
    driver2.povUp().onFalse(climberSub.stopClimb());
    driver2.povDown().onTrue(climberSub.climbDown());
    driver2.povDown().onFalse(climberSub.stopClimb());

    //Dış Asansör
    driver2.povLeft().onTrue(elevatoroOutSub.elevatorUp());
    driver2.povLeft().onFalse(elevatoroOutSub.stopElevatorOut());
    driver2.povRight().onTrue(elevatoroOutSub.elevatorDown());
    driver2.povRight().onFalse(elevatoroOutSub.stopElevatorOut());
  }
  
  public void configurePathPlanner() {
    NamedCommands.registerCommand("intakeAlma", intakeSub.getAutoNote(0.75));
    NamedCommands.registerCommand("NotaAtmaİntake", intakeSub.AmpNote(0.75));
    NamedCommands.registerCommand("NotaAtma", shooterSub.shootNote(0.92));
    NamedCommands.registerCommand("AciAlma", angleSub.SetAngle(-71.5));
    NamedCommands.registerCommand("AsansorAlma", elevatorINSub.SetElevator(-41.5));
    NamedCommands.registerCommand("AciAtis", angleSub.SetAngle(-49.5));
    NamedCommands.registerCommand("AsansorAtis", elevatorINSub.SetElevator(-10));
    NamedCommands.registerCommand("StopShooter", shooterSub.stopShooter());
    NamedCommands.registerCommand("Stopİntake", intakeSub.stopIntake());
    NamedCommands.registerCommand("ShooterBack", shooterSub.shootNote(-0.1));
    drivebase.setupPathPlanner();
  }

  public Command getAutonomousCommand(){
     return new PathPlannerAuto("Mid2Note");
  }

  public void setDriveMode(){}

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  }
}
