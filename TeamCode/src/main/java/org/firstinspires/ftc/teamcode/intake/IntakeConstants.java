package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static double[] STACK_HEIGHTS = {
            140, // 1st cone
            240, // 2nd cone
            340, // 3rd cone
            440, // 4th cone
            540, // 5th cone
    };

    public static double HIGH_JUNCTION_HEIGHT = 2200;
    public static double ON_HIGH_JUNCTION_HEIGHT = 2000;
    public static double MEDIUM_JUNCTION_HEIGHT = 1600;
    public static double ON_MEDIUM_JUNCTION_HEIGHT = 1560;
    public static double LOW_JUNCTION_HEIGHT = 1000;
    public static double ON_LOW_JUNCTION_HEIGHT = 960;
    public static double GROUND_JUNCTION_HEIGHT = 140;
    public static double ON_GROUND_JUNCTION_HEIGHT = 100;
    public static double BOTTOM = 0;

    public static double CLAW_OPEN_POSITION = 0.5;
    public static double CLAW_CLOSED_POSITION = 0.34;
}
