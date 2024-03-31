package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    CANSparkMax intake = new CANSparkMax(22, MotorType.kBrushed);
    public DigitalInput digitalInput = new DigitalInput(8);
    Timer intakeTimer = new Timer();
    
    public IntakeSubsystem(){
    intake.restoreFactoryDefaults();
    intake.setIdleMode(IdleMode.kCoast);
    intake.setInverted(false);
    }

    public Command getNote(double speed){
     return run(()-> {
            if(digitalInput.get() == true){
            intake.set(0);
            } else {
            intake.set(speed);
            }
        });
    }

    public Command getAutoNote(double speed){
     return runOnce(()-> {
            if(digitalInput.get() == true){
            intake.set(0);
            } else {
            intake.set(speed);
            }
        });
    }

    public Command AmpNote(double speed){
     return runOnce(()-> {
            intake.set(speed);
        });
    }

    public Command waitAndShoot(double speed){
     return run(()-> {
             intakeTimer.start();
             if(intakeTimer.get() >= 0.95){
              intake.set(speed);
             }
        });
} 

    public Command stopTeleopIntake(){
     return run(()-> {
            intakeTimer.reset();
            intake.set(0);
        });
} 

    public Command stopIntake(){
     return runOnce(()-> {
            intake.set(0);
        });
} 
}