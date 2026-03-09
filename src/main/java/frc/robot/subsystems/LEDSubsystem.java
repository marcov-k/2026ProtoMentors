package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.field.AllianceUtil;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private int ledlength = 130;
    private int ledpwmport = 0;

    AllianceUtil alliance;

    public LEDSubsystem() {
        led = new AddressableLED(ledpwmport);
        buffer = new AddressableLEDBuffer(ledlength);
        led.setLength(ledlength);
        led.setData(buffer);
        led.start();

        fillColor(255, 255, 255);
    }

    public void fillColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }

    public void autonomousInit() {
        alliance = new AllianceUtil();
        if (alliance.isRed()) {
            fillColor(255, 0,0);
        } 
        else {
            fillColor(0, 0, 255);
        }
    }    

    public void teleopInit() {
        alliance = new AllianceUtil();
        if (alliance.isRed()) {
            fillColor(255, 0,0);
        } 
        else {
            fillColor(0, 0, 255);
        }
    }

    public void disabledInit() {
        fillColor(255, 255,255);
    }
}
