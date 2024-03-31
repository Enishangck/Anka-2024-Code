package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    CANSparkMax shooterMaster = new CANSparkMax(21, MotorType.kBrushless);
    CANSparkMax shooterSlave = new CANSparkMax(23, MotorType.kBrushless);

    public ShooterSubsystem(){
    shooterMaster.restoreFactoryDefaults();
    shooterSlave.restoreFactoryDefaults();

    shooterMaster.setIdleMode(IdleMode.kBrake);
    shooterSlave.setIdleMode(IdleMode.kBrake);

    shooterMaster.setInverted(false);
    shooterSlave.setInverted(false);
    }

    public Command shootNote(double speed){
     return runOnce(()-> {
            shooterMaster.set(speed);
            shooterSlave.set(speed);
        });
    }

    public Command stopShooter(){
     return runOnce(()-> {
            shooterMaster.set(0);
            shooterSlave.set(0);
        });
    }
}
