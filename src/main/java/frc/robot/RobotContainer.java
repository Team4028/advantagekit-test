// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithFlywheelAuto;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonSRX;

import frc.robot.utilities.BeakXBoxController;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Flywheel flywheel;
    private Turret turret;

    // Controller
    private final BeakXBoxController controller = new BeakXBoxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);
    private final LoggedDashboardNumber turretSpeedInput = new LoggedDashboardNumber("Turret Speed", 0.0);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                turret = new Turret(new TurretIOTalonSRX());
                // Use dummy IO for drive and flywheel for now, will remove later.
                drive = new Drive(new DriveIO() {});
                flywheel = new Flywheel(new FlywheelIO() {});
                // drive = new Drive(new DriveIOSparkMax());
                // flywheel = new Flywheel(new FlywheelIOSparkMax());
                // drive = new Drive(new DriveIOFalcon500());
                // flywheel = new Flywheel(new FlywheelIOFalcon500());
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                turret = new Turret(new TurretIOSim());
                drive = new Drive(new DriveIOSim());
                flywheel = new Flywheel(new FlywheelIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                drive = new Drive(new DriveIO() {});
                flywheel = new Flywheel(new FlywheelIO() {});
                break;
        }

        // Set up auto routines
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("Spin", new SpinAuto(drive));
        autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // example code from template
        // drive.setDefaultCommand(new RunCommand(() -> drive.driveArcade(-controller.getLeftY(), controller.getLeftX()), drive));
        // controller.a().whileTrue(new StartEndCommand(() -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

        controller.a.whileTrue(new StartEndCommand(() -> turret.runVoltage(0.3), turret::stop, turret));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
