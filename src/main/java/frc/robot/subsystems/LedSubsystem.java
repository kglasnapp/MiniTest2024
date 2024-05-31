package frc.robot.subsystems;

import static frc.robot.Util.logf;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Robot;

//import static frc.robot.utilities.Util.logf;

public class LedSubsystem extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean change = true;

    // private ElevatorSubsystem elevatorSubsystem;
    // private GrabberTiltSubsystem grabberSubsystem;

    public LedSubsystem() {
        initNeoPixel();
    }

    public enum Leds {
        RobotAlliance(0, 30), 
        GrabberForward(31, 1), 
        GrabberReverse(32, 1), IntakeOverCurrent(33, 3), GrabberOverCurrent(36, 20);

        public final int val;
        public final int number;

        private Leds(int val, int number) {
            this.val = val;
            this.number = number;
        }
    };

    @Override
    public void periodic() {
        // if (Robot.count % 5 == 0) {
        if (change) {
            m_led.setData(m_ledBuffer);
            change = false;
        }

        // }
    }

    private void initNeoPixel() {
        m_led = new AddressableLED(9);
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(64);
        m_led.setLength(m_ledBuffer.getLength());
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setColors(Leds led, int r, int g, int b) {
        for (int i = led.val; i < led.val + led.number; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        change = true;
    }

    public void setLimitSwitchLed(Leds led, boolean value) {
        if (value) {
            setColors(led, 0, 80, 0);
        } else {
            setColors(led, 80, 0, 0);
        }
    }

    public void setAllianceLeds() {
 setColors(Leds.RobotAlliance, 0, 0, 80);

        // if (Robot.alliance.isPresent()) {
        //     if (Robot.alliance.get() == Alliance.Red) {
        //         setColors(Leds.RobotAlliance, 80, 0, 0);
        //     } else {
        //         setColors(Leds.RobotAlliance, 0, 0, 80);
        //     }
        // }
    }

    public void setOverCurrent(Leds led, boolean value) {
        logf("**** set over current led %b\n", value);
        if (value) {
            setColors(led, 80, 0, 0);
        } else {
            setColors(led, 0, 80, 0);
        }
    }
}
