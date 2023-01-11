package org.firstinspires.ftc.teamcode.pathfinding;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

public class Pathfinder {
    List<Obstacle> obstacles;

    public Pathfinder(List<Obstacle> obstacles) {
        this.obstacles = obstacles;
    }

    // Uses a heuristic function to estimate the distance between a node and a goal
    private int heuristicCost(Node node, Node goal) {
        // Manhattan distance
        return Math.abs(node.getX() - goal.getX()) + Math.abs(node.getY() - goal.getY());
    }

    private int edgeWeight(Node start, Node goal) {
        return 1;
    }

    private boolean nodeBlockedByObstacle(Node node) {
        // Give the node to all the obstacle functions
        for (Obstacle o : obstacles) {
            if (o.nodeBlockedByObstacle(node)) {
                return true;
            }
        }

        return false;
    }

    // A function to get the neighbors of a node
    private List<Node> getNeighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        // Delta arrays go from straight up in a clockwise circle
        int[] dx = {0, 1, 1, 1, 0, -1, -1, -1};
        int[] dy = {1, 1, 0, -1, -1, -1, 0, 1};
        for (int i = 0; i < 8; i++) { // 8 is the length of dx and dy
            int x = node.getX() + dx[i];
            int y = node.getY() + dy[i];
            Node newNode = Node.createFromIndex(x, y);
            if (newNode.isInField() && !nodeBlockedByObstacle(newNode)) {
                neighbors.add(newNode);
            }
        }
        return neighbors;
    }

    private List<Node> reconstructPath(HashMap<Node, Node> cameFrom, Node currentNode) {
        List<Node> totalPath = new ArrayList<>();
        while (cameFrom.containsKey(currentNode)) {
            currentNode = cameFrom.get(currentNode);
            totalPath.add(currentNode);
        }
        Collections.reverse(totalPath);
        return totalPath;
    }

    public List<Node> findPath(Node start, Node goal) {
        // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
        HashMap<Node, Node> cameFrom = new HashMap<>();

        // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        HashMap<Node, Integer> gScore = new HashMap<>();
        gScore.put(start, 0);

        // For node n, fScore[n] = gScore[n] + h(n). fScore[n] represents our current best guess as to how cheap a path could be from start to finish if it goes through n.
        HashMap<Node, Integer> fScore = new HashMap<>();
        fScore.put(start, heuristicCost(start, goal));

        // The set of discovered nodes that may need to be (re-)expanded. Initially, only the start node is known.
        // NOTE: Before a node is added to the PriorityQueue it should be added first to fScore
        // Should never throw NullPointerException if the above NOTE is followed
        PriorityQueue<Node> openSet = new PriorityQueue<>(25, (a, b) -> fScore.get(a) - fScore.get(b));
        openSet.add(start);

        // While the queue is not empty
        while (!openSet.isEmpty()) {
            // Remove the node with the lowest cost from the open set
            Node currentNode = openSet.poll();

            // If this is the goal node, we have found the path
            if (currentNode.equals(goal)) {
                return reconstructPath(cameFrom, currentNode);
            }

            // Get the neighbors of this node
            List<Node> neighbors = getNeighbors(currentNode);

            // For each neighbor
            for (Node neighbor : neighbors) {
                // tentativeGScore is the distance from start to the neighbor through current
                // Index currentNode is guaranteed to exist in gScore
                int tentativeGScore = gScore.get(currentNode) + edgeWeight(currentNode, neighbor);

                // Index neighbor is guaranteed to exist because of the first condition
                if (!gScore.containsKey(neighbor) || tentativeGScore < gScore.get(neighbor)) {
                    // This path to neighbor is better than any previous one. Record it!
                    cameFrom.put(neighbor, currentNode);
                    gScore.put(neighbor, tentativeGScore);
                    fScore.put(neighbor, tentativeGScore + heuristicCost(neighbor, goal));
                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }

        // TODO: Handle error
        // No path found
        return null;
    }

    public double[] fitPolynomialToPath(List<Node> path, int polynomialDegree) {
        // Create a curve fitter for polynomial curves with degree 4
        PolynomialCurveFitter fitter = PolynomialCurveFitter.create(polynomialDegree);

        // Add the (x, y) points from the path to the curve fitter
        WeightedObservedPoints obs = new WeightedObservedPoints();
        for (Node node : path) {
            obs.add(node.getXCM(), node.getYCM());
        }

        // Fit the function to the path and return the fitted parameters
        return fitter.fit(obs.toList());
    }
}
