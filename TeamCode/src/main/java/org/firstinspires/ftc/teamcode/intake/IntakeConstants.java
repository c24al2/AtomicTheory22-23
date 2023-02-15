package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static double[] STACK_HEIGHTS = {
            3, // 1st cone
            150, // 2nd cone
            202, // 3rd cone
            305, // 4th cone
            403, // 5th cone
    };

    public static double HIGH_JUNCTION_HEIGHT = 2200;
    public static double MEDIUM_JUNCTION_HEIGHT = 1600;
    public static double LOW_JUNCTION_HEIGHT = 1000;
    public static double GROUND_JUNCTION_HEIGHT = 140;
    public static double BOTTOM = 0;

    public static double ON_JUNCTION_HEIGHT_CHANGE = 300;

    public static double CLAW_OPEN_POSITION = 0.5;
    public static double CLAW_CLOSED_POSITION = 0.35;
}
