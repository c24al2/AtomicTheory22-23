package org.firstinspires.ftc.teamcode.pathfinding;

import static org.firstinspires.ftc.teamcode.Constants.X_NODE_TO_CM;
import static org.firstinspires.ftc.teamcode.Constants.X_RESOLUTION;
import static org.firstinspires.ftc.teamcode.Constants.Y_NODE_TO_CM;
import static org.firstinspires.ftc.teamcode.Constants.Y_RESOLUTION;

public class Node {
    private int x;
    private int y;

    private Node() {}

    private Node(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public static Node createFromIndex(int x, int y) {
        return new Node(x, y);
    }

    public static Node createFromCM(double xCM, double yCM) {
        int x = (int) Math.round(xCM / X_NODE_TO_CM);
        int y = (int) Math.round(yCM / Y_NODE_TO_CM);
        return new Node(x, y);
    }

    public int getX() {
        return x;
    }

    public double getXCM() {
        return x * X_NODE_TO_CM;
    }

    public int getY() {
        return y;
    }

    public double getYCM() {
        return y * Y_NODE_TO_CM;
    }

    public boolean isInField() {
        return x >= 0 && x < X_RESOLUTION && y >= 0 && y < Y_RESOLUTION;
    }

    // Overriding equals() to compare two Node objects by value rather than reference
    public boolean equals(Object o) {
        // Check if o is an instance of Node or not
        if (!(o instanceof Node)) {
            return false;
        }

        // If the object is compared with itself then return true
        if (o == this) {
            return true;
        }

        // typecast o to Complex so that we can compare data members
        Node node = (Node) o;

        // Compare the data members and return accordingly
        return this.x == node.x && this.y == node.y;
    }

    // Need to override as we override equals as well
    public int hashCode() {
        int hash = 17;
        hash = 31 * hash + this.x;
        hash = 31 * hash + this.y;
        return hash;
    }
}