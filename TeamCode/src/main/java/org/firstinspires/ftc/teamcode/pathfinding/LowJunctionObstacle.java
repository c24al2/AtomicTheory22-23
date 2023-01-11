package org.firstinspires.ftc.teamcode.pathfinding;

import static org.firstinspires.ftc.teamcode.Constants.LOW_JUNCTION_OBSTACLE_RADIUS;

public class LowJunctionObstacle implements Obstacle {
    Node center;

    public LowJunctionObstacle(Node center) {
        this.center = center;
    }

    // Coordinates are given in CM
    @Override
    public boolean nodeBlockedByObstacle(Node node) {
        double dx = node.getXCM()-center.getXCM();
        double dy = node.getYCM()-center.getYCM();
        double r2 = LOW_JUNCTION_OBSTACLE_RADIUS * LOW_JUNCTION_OBSTACLE_RADIUS;
        return dx*dx + dy*dy <= r2;
    }
}
