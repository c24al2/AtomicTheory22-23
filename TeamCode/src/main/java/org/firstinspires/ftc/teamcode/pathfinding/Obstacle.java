package org.firstinspires.ftc.teamcode.pathfinding;

public interface Obstacle {
    // Both x and y should be given in CM
    boolean nodeBlockedByObstacle(Node node);
}
