package org.firstinspires.ftc.teamcode.pathfinding;

import static org.firstinspires.ftc.teamcode.Constants.GROUND_JUNCTION_OBSTACLE_RADIUS;

public class GroundJunctionObstacle implements Obstacle {
    Node center;

    public GroundJunctionObstacle(Node center) {
        this.center = center;
    }

    // Coordinates are given in CM
    @Override
    public boolean nodeBlockedByObstacle(Node node) {
        double dx = node.getXCM()-center.getXCM();
        double dy = node.getYCM()-center.getYCM();
        double r2 = GROUND_JUNCTION_OBSTACLE_RADIUS * GROUND_JUNCTION_OBSTACLE_RADIUS;
        return dx*dx + dy*dy <= r2;
    }
}
