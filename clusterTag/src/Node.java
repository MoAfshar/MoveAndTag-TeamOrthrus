public class Node {
    int nodeNum1;
    int nodeNum2;
    public int numRobotsVisited;
    public boolean visitedTwo;
    Node parent;
    Node leftChild = null;
    Node rightChild = null;
    public double current_dist = 0;

    public Node (int nodeNum, Node parent, double dist) {
        this.nodeNum1 = nodeNum;
        this.parent = parent;
        this.visitedTwo = false;
        if (parent!=null) numRobotsVisited = parent.numRobotsVisited + 1;
        else numRobotsVisited = 1;
        this.current_dist += dist;
    }

    public Node (int nodeNum1, int nodeNum2, Node parent, double dist) {
        this.nodeNum1 = nodeNum1;
        this.nodeNum2 = nodeNum2;
        this.visitedTwo = true;
        this.parent = parent;
        this.visitedTwo = true;
        if (parent!=null) numRobotsVisited = parent.numRobotsVisited + 2;
        else numRobotsVisited = 1;
        this.current_dist += dist;
    }

    public void addLeft (Node leftChild) {
        this.leftChild = leftChild;
    }

    public void addRight (Node rightChild) {
        this.rightChild = rightChild;
    }
//    public VisitingNodes makeCopy () {
//        if (visitedTwo) return new VisitingNodes(nodeNum, node parent);
//    }

}