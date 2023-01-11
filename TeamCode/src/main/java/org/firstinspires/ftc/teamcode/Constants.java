package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pathfinding.GroundJunctionObstacle;
import org.firstinspires.ftc.teamcode.pathfinding.HighJunctionObstacle;
import org.firstinspires.ftc.teamcode.pathfinding.LowJunctionObstacle;
import org.firstinspires.ftc.teamcode.pathfinding.MediumJunctionObstacle;
import org.firstinspires.ftc.teamcode.pathfinding.Node;
import org.firstinspires.ftc.teamcode.pathfinding.Obstacle;

public class Constants {
    public static final double STICK_THRESH = 0.2;
    public static final double ROTATION_SENSITIVITY = 0.5;
    public static final int CAM_WIDTH = 1280;
    public static final int CAM_HEIGHT = 720;

    // Configuration variables for AprilTags
    public static final double TAG_SIZE = 0.038;  // Units are meters
    public static final String TAG_FAMILY = "tag25h9";  // Chose 25h9 tags because they have the largest data bits but still have a relatively high hemming distance
    public static final float DECIMATION = 1.0f;  // Increasing decimation speeds up tag finding, but might reduce accuracy/reliability
    public static final int THREADS = 3;  // Number of threads to use to scan for the AprilTag
    public static final double FX = 1.39747644e+03;  // The camera's horizontal focal length in pixels (don't change unless using new camera)
    public static final double FY = 1.39191699e+03 ;  // The camera's vertical focal length in pixels (don't change unless using new camera)
    public static final double CX = 9.64795834e+02;  // The camera's horizontal focal center in pixels (don't change unless using new camera)
    public static final double CY = 5.09429377e+02;  // The camera's vertical focal center in pixels (don't change unless using new camera)

    // Configuration variables for AStar
    public static final int X_RESOLUTION = 144;
    public static final int Y_RESOLUTION = 144;
    public static final double X_NODE_TO_CM = 365.76 / X_RESOLUTION;
    public static final double Y_NODE_TO_CM = 365.76 / Y_RESOLUTION;
    public static final double HIGH_JUNCTION_OBSTACLE_RADIUS = 22; // Units are CM
    public static final double MEDIUM_JUNCTION_OBSTACLE_RADIUS = 22; // Units are CM
    public static final double LOW_JUNCTION_OBSTACLE_RADIUS = 22; // Units are CM
    public static final double GROUND_JUNCTION_OBSTACLE_RADIUS = 24; // Units are CM
    public static final double FT_TO_CM = 30.48;
    public static final Obstacle[] FIELD_OBSTACLES = {
            // FIRST ROW
            new GroundJunctionObstacle(Node.createFromCM(2*FT_TO_CM, 10*FT_TO_CM)),
            new LowJunctionObstacle(Node.createFromCM(4*FT_TO_CM, 10*FT_TO_CM)),
            new GroundJunctionObstacle(Node.createFromCM(6*FT_TO_CM, 10*FT_TO_CM)),
            new LowJunctionObstacle(Node.createFromCM(8*FT_TO_CM, 10*FT_TO_CM)),
            new GroundJunctionObstacle(Node.createFromCM(10*FT_TO_CM, 10*FT_TO_CM)),
            // SECOND ROW
            new LowJunctionObstacle(Node.createFromCM(2*FT_TO_CM, 8*FT_TO_CM)),
            new MediumJunctionObstacle(Node.createFromCM(4*FT_TO_CM, 8*FT_TO_CM)),
            new HighJunctionObstacle(Node.createFromCM(6*FT_TO_CM, 8*FT_TO_CM)),
            new MediumJunctionObstacle(Node.createFromCM(8*FT_TO_CM, 8*FT_TO_CM)),
            new LowJunctionObstacle(Node.createFromCM(10*FT_TO_CM, 8*FT_TO_CM)),
            // THIRD ROW
            new GroundJunctionObstacle(Node.createFromCM(2*FT_TO_CM, 6*FT_TO_CM)),
            new HighJunctionObstacle(Node.createFromCM(4*FT_TO_CM, 6*FT_TO_CM)),
            new GroundJunctionObstacle(Node.createFromCM(6*FT_TO_CM, 6*FT_TO_CM)),
            new HighJunctionObstacle(Node.createFromCM(8*FT_TO_CM, 6*FT_TO_CM)),
            new GroundJunctionObstacle(Node.createFromCM(10*FT_TO_CM, 6*FT_TO_CM)),
            // FOURTH ROW
            new LowJunctionObstacle(Node.createFromCM(2*FT_TO_CM, 4*FT_TO_CM)),
            new MediumJunctionObstacle(Node.createFromCM(4*FT_TO_CM, 4*FT_TO_CM)),
            new HighJunctionObstacle(Node.createFromCM(6*FT_TO_CM, 4*FT_TO_CM)),
            new MediumJunctionObstacle(Node.createFromCM(8*FT_TO_CM, 4*FT_TO_CM)),
            new LowJunctionObstacle(Node.createFromCM(10*FT_TO_CM, 4*FT_TO_CM)),
            // FIFTH ROW
            new GroundJunctionObstacle(Node.createFromCM(2*FT_TO_CM, 2*FT_TO_CM)),
            new LowJunctionObstacle(Node.createFromCM(4*FT_TO_CM, 2*FT_TO_CM)),
            new GroundJunctionObstacle(Node.createFromCM(6*FT_TO_CM, 2*FT_TO_CM)),
            new LowJunctionObstacle(Node.createFromCM(8*FT_TO_CM, 2*FT_TO_CM)),
            new GroundJunctionObstacle(Node.createFromCM(10*FT_TO_CM, 2*FT_TO_CM)),
    };
}
