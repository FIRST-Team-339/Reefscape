// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private AddressableLED led = new AddressableLED(1);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(100);
    private Distance ledSpacing = Meters.of(5.0 / 300);

    private LEDPattern rainbow =
            LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private LEDPattern teleopAligned =
            LEDPattern.gradient(GradientType.kContinuous, Color.kYellow, Color.kOrange)
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private LEDPattern coralGrabbed =
            LEDPattern.gradient(GradientType.kContinuous, Color.kLightSkyBlue, Color.kAquamarine)
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);

    public static enum LEDMode {
        Default,
        TeleopAligned,
        CoralGrabbed
    }

    private LEDMode mode = LEDMode.Default;

    /** Creates a new LEDs. */
    public LEDs() {
        this.led.setLength(this.ledBuffer.getLength());
        this.led.setData(this.ledBuffer);
        this.led.start();

        // this.rainbow.applyTo(this.ledBuffer);
    }

    public void setMode(LEDMode newMode) {
        this.mode = newMode;
    }

    @Override
    public void periodic() {
        switch (mode) {
            case Default:
                this.rainbow.applyTo(ledBuffer);
                break;
            case TeleopAligned:
                this.teleopAligned.applyTo(ledBuffer);
                break;
            case CoralGrabbed:
                this.coralGrabbed.applyTo(ledBuffer);
                break;
        }
        this.led.setData(ledBuffer);
    }
}
