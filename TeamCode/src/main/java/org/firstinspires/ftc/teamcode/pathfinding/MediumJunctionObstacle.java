package org.firstinspires.ftc.teamcode.pathfinding;

import static org.firstinspires.ftc.teamcode.Constants.MEDIUM_JUNCTION_OBSTACLE_RADIUS;

public class MediumJunctionObstacle implements Obstacle {
    Node center;

    public MediumJunctionObstacle(Node center) {
        this.center = center;
    }

    // Coordinates are given in CM
    @Override
    public boolean nodeBlockedByObstacle(Node node) {
        double dx = node.getXCM()-center.getXCM();
        double dy = node.getYCM()-center.getYCM();
        double r2 = MEDIUM_JUNCTION_OBSTACLE_RADIUS * MEDIUM_JUNCTION_OBSTACLE_RADIUS;
        return dx*dx + dy*dy <= r2;
    }
}
