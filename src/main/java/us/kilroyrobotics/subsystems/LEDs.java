// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(38);
    private Distance ledSpacing = Meters.of(5.0 / 300);

    private LEDPattern rainbow =
            LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private LEDPattern teleopAligned =
            LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kLimeGreen)
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private LEDPattern waitingForCoral =
            LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kForestGreen)
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private LEDPattern coralGrabbed =
            LEDPattern.gradient(GradientType.kContinuous, Color.kLightSkyBlue, Color.kIndigo)
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private LEDPattern defense =
            LEDPattern.gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kDarkRed)
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), ledSpacing);

    public static enum LEDMode {
        Off,
        Rainbow,
        TeleopAligned,
        WaitingForCoral,
        CoralDetected,
        Defense
    }

    private LEDMode mode = LEDMode.Off;

    /** Creates a new LEDs. */
    public LEDs() {
        led.setColorOrder(ColorOrder.kBRG);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public LEDMode getMode() {
        return mode;
    }

    public void setMode(LEDMode newMode) {
        mode = newMode;
    }

    @Override
    public void periodic() {
        switch (mode) {
            case Rainbow:
                rainbow.applyTo(ledBuffer);
                break;
            case TeleopAligned:
                teleopAligned.applyTo(ledBuffer);
                break;
            case WaitingForCoral:
                waitingForCoral.applyTo(ledBuffer);
                break;
            case CoralDetected:
                coralGrabbed.applyTo(ledBuffer);
                break;
            case Defense:
                this.defense.applyTo(ledBuffer);
                break;
            default:
                LEDPattern.kOff.applyTo(ledBuffer);
                break;
        }
        led.setData(ledBuffer);
    }
}
