package org.firstinspires.ftc.teamcode;

public class MatchState {
    public static enum AllianceColor {
    RED,
    BLUE
}

    // This variable persists across OpModes
    public static AllianceColor selectedColor = AllianceColor.RED;
}
