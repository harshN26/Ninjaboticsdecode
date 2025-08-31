package org.firstinspires.ftc.teamcode.Commands.BasicCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.utils.LEDs;

public class LEDCommand extends InstantCommand {
    public LEDCommand(LEDs leds, RevBlinkinLedDriver.BlinkinPattern pattern){
        super(
                ()->leds.update(pattern)
        );
    }
}
