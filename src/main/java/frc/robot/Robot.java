package frc.robot;

// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * 3ft: 60deg 0.5 power
 * 4ft: 55deg 0.5 power
 * 6ft: 50deg 0.7 power
 * 7ft: 44deg 0.7 power
 * 8ft: 42deg 0.7 power
*/
public class Robot extends TimedRobot {
  CANSparkBase motorR = new CANSparkFlex(1, MotorType.kBrushless);
  CANSparkBase motorL = new CANSparkFlex(2, MotorType.kBrushless);
  //CANcoder canCoder = new CANcoder(9, "RIO");

  XboxController cont = new XboxController(2);

  boolean voltageControl = false;

  @Override
  public void robotInit() {
    motorL.getPIDController().setP(0.1);
    // motorL.getPIDController().setD(0.2);
    motorR.getPIDController().setP(0.1);
    // motorR.getPIDController().setD(0.2);
  }

  @Override
  public void teleopPeriodic() {
    int pov = cont.getPOV();
    double value = 0.0;

    if (pov == 0) {
      value = 0.1;
      // value = 2000;
    } else if (pov == 90) {
      value = 0.25;
      // value = 3000;
    } else if (pov == 180) {
      value = 0.5;
      // value = 4000;
    } else if (pov == 270) {
      value = 0.7;
      // value = 5000;
    }

    motorL.set(value);
    motorR.set(value);

    // motorL.getPIDController().setReference(value,
    // CANSparkBase.ControlType.kVelocity);
    // motorR.getPIDController().setReference(value,
    // CANSparkBase.ControlType.kVelocity);
  }
}