package MoveAndTagRobots;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

public class Algorithm {
	// global variables
	private double[][] floydTable;
	private int[][] warshalls;
	private double epsilon = 0.000000001;

	private Point[] allPoints;
	private boolean allFound = false;
	public static final double MAXVAL = Double.MAX_VALUE / 2 - 1;
	private ArrayList<ArrayList<Point>> allPaths = new ArrayList<>();

	private int totalRobots;
	private int robotsFound;
	
	PriorityQueue<RobotEdge> edges;

	public static double calculateDistance(Point p1, Point p2) {
		return Math.sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y)
				* (p2.y - p1.y));
	}

	public double findY(Point p1, Point p2, double new_x) {
		double x1 = p1.x;
		double y1 = p1.y;
		double x2 = p2.x;
		double y2 = p2.y;
		double current_m = (y2 - y1) / (x2 - x1);
		double current_c = y1 - (x1 * current_m);
		return (current_m * new_x) + current_c;
	}

	public boolean inObstacle(Point p, Point[] obstacle) {
		int len = obstacle.length;
		double x = p.x;
		double y = p.y;
		boolean oddNodes = false;
		for (int i = 0; i < len - 1; i++) {
			double xi = obstacle[i].x;
			double yi = obstacle[i].y;
			double xj = obstacle[i + 1].x;
			double yj = obstacle[i + 1].y;
			if ((yi < y && yj > y) || (yj < y && yi > y)) {
				if (xi + (y - yi) / (yj - yi) * (xj - xi) < x) {
					oddNodes = !oddNodes;
				}
			}
		}
		double xi = obstacle[0].x;
		double yi = obstacle[0].y;
		double xj = obstacle[len - 1].x;
		double yj = obstacle[len - 1].y;
		if ((yi < y && yj > y) || (yj < y && yi > y)) {
			if (xi + (y - yi) / (yj - yi) * (xj - xi) < x) {
				oddNodes = !oddNodes;
			}
		}

		return oddNodes;
	}

	public boolean pathCollidesObstacle(Point p1, Point p2, Point[] obstacle) {
		for (int i = 0; i < obstacle.length - 1; i++) {
			Intersection intersect = isIntersecting(p1, p2, obstacle[i],
					obstacle[i + 1]);
			if (intersect == Intersection.YES) {
				return true;
			}
			if (intersect == Intersection.P1SAME) {
				double new_x = p1.x < p2.x ? p1.x + epsilon : p1.x - epsilon;
				double new_y = findY(p1, p2, new_x);
				if (inObstacle(new Point(new_x, new_y), obstacle)) {
					return true;
				}
			}
			if (intersect == Intersection.P2SAME) {
				double new_x = p2.x < p1.x ? p2.x + epsilon : p2.x - epsilon;
				double new_y = findY(p1, p2, new_x);
				if (inObstacle(new Point(new_x, new_y), obstacle)) {
					return true;
				}
			}
		}

		return false;

	}

	public enum Intersection {
		NO, YES, COLLINEAR, P1SAME, P2SAME
	}

	public double crossProduct(Point p1, Point p2) {
		return p1.x * p2.y - p2.x * p1.y;
	}

	public boolean isPointOnLine(Point p1, Point p2, Point p3) {
		Point b = new Point(p2.x - p1.x, p2.y - p1.y);
		Point bTmp = new Point(p3.x - p1.x, p3.y - p1.y);
		double r = crossProduct(b, bTmp);
		return Math.abs(r) < epsilon;
	}

	public boolean isPointRightOfLine(Point p1, Point p2, Point p3) {
		Point b = new Point(p2.x - p1.x, p2.y - p1.y);
		Point bTmp = new Point(p3.x - p1.x, p3.y - p1.y);
		return crossProduct(b, bTmp) < 0;
	}

	public boolean lineTouchesOrCrossesLine(Point p1, Point p2, Point p3,
			Point p4) {
		return isPointOnLine(p1, p2, p3)
				|| isPointOnLine(p1, p2, p4)
				|| (isPointRightOfLine(p1, p2, p3) ^ isPointRightOfLine(p1, p2,
						p4));
	}

	public Intersection isIntersecting(Point p1, Point p2, Point p3, Point p4) {
		if (p1.equals(p3) || p1.equals(p4)) {
			return Intersection.P1SAME;
		}

		if (p2.equals(p3) || p2.equals(p4)) {
			return Intersection.P2SAME;
		}

		if (lineTouchesOrCrossesLine(p1, p2, p3, p4)
				&& lineTouchesOrCrossesLine(p3, p4, p1, p2)) {
			return Intersection.YES;
		}

		return Intersection.NO;
	}

	public void calculatePointVisibility(Point[] allPoints,
			ArrayList<Point[]> obstaclePoints) {
		int len = allPoints.length;
		ArrayList<Point> visiblePoints = new ArrayList<>();
		boolean collides = false;
		for (int i = 0; i < len; i++) {
			for (int j = 0; j < len; j++) {
				if (j != i) {
					for (Point[] obstaclePoint : obstaclePoints) {
						if (pathCollidesObstacle(allPoints[i], allPoints[j],
								obstaclePoint)) {
							collides = true;
							break;
						}
					}
					if (!collides)
						visiblePoints.add(allPoints[j]);
					collides = false;
				}
			}
			allPoints[i].setVisible(visiblePoints);
			visiblePoints = new ArrayList<>();
		}
	}

	public void init(int pointNum) {
		this.allPoints = new Point[pointNum]; 

		floydTable = new double[pointNum][pointNum];
		warshalls = new int[pointNum][pointNum];
		for (int i = 0; i < pointNum; i++) {
			for (int j = 0; j < pointNum; j++) {
				warshalls[i][j] = -1;
			}
		}
	}

	public void calcFloydDistances(Point[] points) {
		int len = points.length;
		for (int i = 0; i < len; i++) {
			for (int j = 0; j < len; j++) {
				if (i == j) {
					floydTable[i][j] = 0;
					warshalls[i][j] = -3;
				} else if (points[i].isVisible(points[j])) {
					floydTable[i][j] = calculateDistance(points[i], points[j]);
					warshalls[i][j] = -2;
				} else {
					floydTable[i][j] = MAXVAL;
				}
			}
		}
	}

	public void calcFloydPaths(Point[] points) {
		for (int k = 0; k < points.length; k++) {
			for (int i = 0; i < points.length; i++) {
				for (int j = 0; j < points.length; j++) {
					if (floydTable[i][k] + floydTable[k][j] < floydTable[i][j]) {
						floydTable[i][j] = floydTable[i][k] + floydTable[k][j];
						warshalls[i][j] = k;
					}
				}
			}
		}
	}

	public String path(int x, int y, Point[] points) {
		int k = warshalls[x][y];
		if (k == -1)
			return "";// points[x].toString() + " cannot be reached from " +
						// points[y].toString();
			// else System.out.println("K IS " + k);
		if (k == -2)
			return points[y].toString() + ", ";
		if (k == -3)
			return "";// "same point!";
		return path(x, k, points) + path(k, y, points);
	}

	public int findNearest(int current, Point[] points, int numRobots) {
		double min_dist = MAXVAL;
		int min_index = current;
		for (int i = 1; i < numRobots; i++) {
			if (!points[i].hasWoken) {
				if (min_dist > floydTable[current][i]) {
					min_dist = floydTable[current][i];
					min_index = i;
				}
			}
		}
		points[min_index].hasWoken = true;
		return min_index;
	}

	public double[] findNearestAndDist(int current, int numRobots,
			boolean[] wokenUp) {
		double[] map = new double[2];
		double min_dist = MAXVAL;
		int min_index = current;
		for (int i = 0; i < numRobots; i++) {
			if (!wokenUp[i]) {
				if (min_dist > floydTable[current][i]) {
					min_dist = floydTable[current][i];
					min_index = i;
				}
			}
		}
		allPoints[min_index].hasWoken = true;
		map[0] = min_index;
		map[1] = min_dist;
		return map;
	}

	public String visitAll(Point[] points, int numRobots) {
		int numAwaken = 1;
		String output = points[0].toString() + ", ";

		int current = 0;
		while (numAwaken <= numRobots) {
			int next = findNearest(current, points, numRobots);
			String pathStr = path(current, next, points);
			if (!pathStr.isEmpty()) {
				output = output + pathStr;
			}
			numAwaken++;
			current = next;
		}

		return output;
	}

	public String allRobotsVisit(Point[] points, int numRobots) {
		Node root = new Node(0, null, 0);
		allPoints = points;
		ArrayList<Point> path = new ArrayList<>();
		boolean[] awaken = new boolean[numRobots];
		awaken[0] = true;
		for (int i = 1; i < numRobots; i++)
			awaken[i] = false;
		Node endNode = recursiveVisit(root, numRobots, awaken);
		buildPaths(endNode, path);
		String finalSolution = "";

		for (ArrayList<Point> singlePath : allPaths) {
			int size = singlePath.size();
			Point[] singlePathArray = new Point[size];
			int index = 0;
			for (int i = size - 1; i >= 0; i--) {
				singlePathArray[index++] = singlePath.get(i);
				// System.out.println("SinglePathArray " + (index-1) + " = " +
				// singlePathArray[index-1]);
			}
			String solution = visitAll(singlePathArray, size);
			solution = solution.substring(0, solution.length() - 3);
			solution += "; ";
			// System.out.println("SOLUTION IS " + solution);
		}

		return finalSolution;
	}

	public void buildPaths(Node node, ArrayList<Point> path) {
		if (node == null) {
			allPaths.add(path);
		} else {
			if (!node.visitedTwo) {
				path.add(allPoints[node.nodeNum1]);
				// System.out.println("Adding " +
				// globalPoints[node.nodeNum1].toString() + " to path");
				buildPaths(node.parent, path);
			} else {
				// System.out.println("IN ELSE");
				path.add(allPoints[node.nodeNum1]);
				// System.out.println("Adding " +
				// globalPoints[node.nodeNum1].toString() + " to path");
				ArrayList<Point> newPath = new ArrayList<>();
				for (Point p : path) {
					newPath.add(p);
				}
				newPath.add(allPoints[node.nodeNum2]);
				// System.out.println("Adding " +
				// globalPoints[node.nodeNum2].toString() + " to newPath");
				allPaths.add(newPath);
				buildPaths(node.parent, path);
			}
		}

	}

	public Node recursiveVisit(Node root, int numRobots, boolean[] awaken) {
		if (allFound)
			return null;
		// System.out.print("Recursively visiting with ");
		int current = root.nodeNum1;
		double[] map = findNearestAndDist(current, numRobots, awaken);
		int nearest1 = (int) map[0];
		System.out.print(nearest1 + " and ");
		double dist = map[1];
		Node leftChild = new Node(nearest1, root, dist);
		root.addLeft(leftChild);
		if (leftChild.numRobotsVisited == numRobots) {
			allFound = true;
			// System.out.println("returning left child");
			return leftChild;
		}
		awaken[nearest1] = true;
		double[] map2 = findNearestAndDist(current, numRobots, awaken);
		int nearest2 = (int) map2[0];
		// System.out.println(nearest1 + " with " + nearest2);
		double dist2 = Math.max(dist, map2[1]);
		// System.out.println("Left child num visited is " +
		// leftChild.numRobotsVisited);
		Node rightChild = new Node(nearest1, nearest2, root, dist2);
		root.addRight(rightChild);
		// System.out.println("Right child num visited is " +
		// rightChild.numRobotsVisited);
		if (rightChild.numRobotsVisited == numRobots) {
			allFound = true;
			// System.out.println("returning right child because " +
			// rightChild.numRobotsVisited + " == " + numRobots);
			return rightChild;
		}
		// System.out.println("");
		Node leftReturn = recursiveVisit(leftChild, numRobots, awaken);
		awaken[nearest2] = true;
		Node rightReturn = recursiveVisit(rightChild, numRobots, awaken);
		return rightReturn == null ? leftReturn : rightReturn;
	}

	public double[] findNearestPoint(int current) {
		double[] res = new double[2];
		double min_dist = MAXVAL;
		int min_index = current;
		for (int i = 0; i < totalRobots; i++) {
			if (!allPoints[i].hasWoken) {
				if (min_dist > floydTable[current][i]) {
					min_dist = floydTable[current][i];
					min_index = i;
				}
			}
		}
		allPoints[min_index].hasWoken = true;
		res[0] = min_index;
		res[1] = min_dist;
		return res;
	}

	public String callBuildTree(Point[] points, int numRobots) {
		allPoints = points;
		totalRobots = numRobots;
		PriorityQueue<RobotEdge> edges = new PriorityQueue<RobotEdge>() {
			public int compare(RobotEdge o1, RobotEdge o2) {
				return (int) (o1.totalDistance - o2.totalDistance);
			}
		};
		double[] nearestArray = findNearestPoint(0);
		int nearest = (int) nearestArray[0];
		double dist = nearestArray[1];
		RobotEdge first_elem = new RobotEdge(0, nearest, dist);
		edges.add(first_elem);
		ArrayList<RobotEdge> savedEdges = new ArrayList<>();

		while (!edges.isEmpty()) {
			RobotEdge current = edges.poll();
			savedEdges.add(current);
			if (robotsFound == totalRobots) {
				continue;
			}
			double[] nearest1 = findNearestPoint(current.source);
			int nearest1_index = (int) nearest1[0];
			if (nearest1_index != current.destination) {
				double nearest1_dist = nearest1[1];
				edges.add(new RobotEdge(current.source, nearest1_index,
						current.totalDistance + nearest1_dist));
			}
			robotsFound++;
			if (robotsFound == totalRobots) {
				continue;
			}
			double[] nearest2 = findNearestPoint(current.destination);
			int nearest2_index = (int) nearest2[0];
			if (nearest2_index != current.destination) {
				double nearest2_dist = nearest2[1];
				edges.add(new RobotEdge(current.destination, nearest2_index,
						current.totalDistance + nearest2_dist));
			}
			robotsFound++;
		}
		ArrayList<ArrayList<Integer>> chosenPaths = new ArrayList<>(
				totalRobots);
		for (int i = 0; i < totalRobots; i++) {
			ArrayList<Integer> chosenPath = new ArrayList<>();
			chosenPath.add(i);
			int elemSize = savedEdges.size();
			for (int j = 0; j < elemSize; j++) {
				RobotEdge elem = savedEdges.get(j);
				if (elem.source == i) {
					chosenPath.add(elem.destination);
				}
			}
			if (chosenPath.size() > 1)
				chosenPaths.add(chosenPath);
		}

		String result = "";

		for (ArrayList<Integer> path : chosenPaths) {
			if (path == null || path.size() == 0)
				continue;
			String current_path = getRobotString(path);
			result += current_path + "; ";
		}

		return result.substring(0, result.length() - 2);
	}

	public String path_new(int x, int y) {
		int k = warshalls[x][y];
		if (k == -1)
			return "";// globalPoints[x].toString() + " cannot be reached from "
						// + globalPoints[y].toString();
			// else System.out.println("K IS " + k);
		if (k == -2)
			return allPoints[y].toString() + ", ";
		if (k == -3)
			return "";// "same point!";
		return path_new(x, k) + path_new(k, y);
	}

	public String getRobotString(ArrayList<Integer> pointsIndex) {
		String output = allPoints[pointsIndex.get(0)].toString() + ", ";
		
		for (int i = 1; i < pointsIndex.size(); i++) {
			output = output
					+ path_new(pointsIndex.get(i - 1), pointsIndex.get(i));
		}
		return output.substring(0, output.length() - 2);
	}
	
	public String getRobotString(ArrayList<Integer> pointsIndex, Point[] robotPoints) {
		String output = findRobotByIndex(pointsIndex.get(0), robotPoints).toString() + ", ";
		
		for (int i = 1; i < pointsIndex.size(); i++) {
			output = output
					+ path_new(pointsIndex.get(i - 1), pointsIndex.get(i));
		}
		return output.substring(0, output.length() - 2);
	}

	public String runClusterAlgorithm(int k, Point[] points, Point[] robotPoints){ 
		//chosenPaths = new ArrayList<>(robotPoints.length);
		allPoints = points;

		edges = new PriorityQueue<RobotEdge>() {
			public int compare(RobotEdge o1, RobotEdge o2) {
				return (int) (o1.totalDistance - o2.totalDistance);
			}
		};
		
		HashMap<Integer, Cluster> clusters = generateClusters(k, robotPoints, floydTable);
		Cluster firstCluster = null; //contains start robot
		for(Map.Entry<Integer, Cluster> entry : clusters.entrySet()){
			Cluster c = entry.getValue();
			if(c.containsFirstRobot()){
				firstCluster = c;
				break;
			}
		}

		//find the first robot in the first cluster
		Point[] robots = firstCluster.getRobotPoints();
		Point firstRobot = null;
		for(int i = 0; i < robots.length; i++){
			if(robots[i].index == 0){
				firstRobot = robots[i];
			}
		}

		//wake up robots in all clusters starting with the first
		return wakeUpRobotsInCluster(firstRobot, points, robotPoints.length, clusters);

	}

	public String wakeUpRobotsInCluster(Point firstRobot, Point[] allPoints, int numRobots, HashMap<Integer, Cluster> allClusters){

		int firstRobotClusterID = firstRobot.cluster;
		Cluster firstCluster = allClusters.get(firstRobotClusterID);
		firstCluster.robotsWoken = 1;
		firstCluster.claimed = true;
		
		//first try waking up the robot at the centre of the cluster
		if(!firstCluster.centreOfMassPoint.hasWoken){
			int nearest = firstCluster.centreOfMassPoint.index;
			double dist = floydTable[firstRobot.index][nearest];
			RobotEdge first_elem = new RobotEdge(firstRobotClusterID, firstRobotClusterID, firstRobot.index, nearest, dist);
			edges.add(first_elem);
		}else{
			double[] nearestArray = findNearestPointInCluster(firstRobot.index, firstCluster);
			
			if(nearestArray != null){
				int nearest = (int) nearestArray[0];
				double dist = nearestArray[1];
				RobotEdge first_elem = new RobotEdge(firstRobotClusterID, firstRobotClusterID, firstRobot.index, nearest, dist);
				edges.add(first_elem);
			}
		}

		ArrayList<RobotEdge> savedEdges = new ArrayList<>();

		while (!edges.isEmpty()) {
			RobotEdge current = edges.poll();
			allClusters.get(current.sourceClusterID).totalRobots = allClusters.get(current.sourceClusterID).robots.size();
			allClusters.get(current.sourceClusterID).claimed = true;
			allClusters.get(current.sourceClusterID).robotsWoken++;
			
			savedEdges.add(current);
			
			if (allClusters.get(current.sourceClusterID).robotsWoken == allClusters.get(current.sourceClusterID).totalRobots) {
				continue;
			}

			double[] nearest1 = findNearestPointInCluster(current.source, allClusters.get(current.sourceClusterID));
			double[] nearest2 = findNearestPointInCluster(current.destination, allClusters.get(current.destinationClusterID));
			
			if(nearest1 == null){
				Point robotPoint = findRobotByIndex(current.source, allClusters.get(current.sourceClusterID).getRobotPoints());
				Cluster nextClosestCluster = getNearestCluster(robotPoint, allClusters);
				Point clusterCenter = nextClosestCluster.centreOfMassPoint;
				if(!clusterCenter.hasWoken){
					edges.add(new RobotEdge(current.sourceClusterID, nextClosestCluster.clusterID, current.source, clusterCenter.index,
						current.totalDistance + floydTable[current.source][clusterCenter.index]));
				}else{
					double[] nearestInCluster = findNearestPointInCluster(current.source, nextClosestCluster);
					int nearestC_index = (int) nearestInCluster[0];
					edges.add(new RobotEdge(current.sourceClusterID, nextClosestCluster.clusterID, current.source, nearestC_index,
							current.totalDistance + floydTable[current.source][nearestC_index]));
				}
			}else {
				int nearest1_index = (int) nearest1[0];
				if (nearest1_index != current.destination) {
					double nearest1_dist = nearest1[1];
					edges.add(new RobotEdge(current.sourceClusterID, current.sourceClusterID, current.source, nearest1_index,
							current.totalDistance + nearest1_dist));
				}
			}
			if(nearest2 == null){
				Point robotPoint = findRobotByIndex(current.destination, allClusters.get(current.destinationClusterID).getRobotPoints());
				Cluster nextClosestCluster = getNearestCluster(robotPoint, allClusters);
				Point clusterCenter = nextClosestCluster.centreOfMassPoint;
				if(!clusterCenter.hasWoken){
					edges.add(new RobotEdge(current.sourceClusterID, nextClosestCluster.clusterID, current.destination, clusterCenter.index,
						current.totalDistance + floydTable[current.destination][clusterCenter.index]));
				}else{
					double[] nearestInCluster = findNearestPointInCluster(current.destination, nextClosestCluster);
					int nearestC_index = (int) nearestInCluster[0];
					edges.add(new RobotEdge(current.destinationClusterID, nextClosestCluster.clusterID, current.destination, nearestC_index,
							current.totalDistance + floydTable[current.destination][nearestC_index]));
				}
			}else {
				int nearest2_index = (int) nearest2[0];
				if (nearest2_index != current.destination) {
					double nearest2_dist = nearest2[1];
					edges.add(new RobotEdge(current.destinationClusterID, current.destinationClusterID, current.destination, nearest2_index,
							current.totalDistance + nearest2_dist));
				}
			}
		}
		
		ArrayList<ArrayList<Integer>> chosenPaths = new ArrayList<>(numRobots);
		for (int i = 0; i < numRobots; i++) {
			ArrayList<Integer> chosenPath = new ArrayList<>();
			chosenPath.add(i);
			int elemSize = savedEdges.size();
			for (int j = 0; j < elemSize; j++) {
				RobotEdge elem = savedEdges.get(j);
				if (elem.source == i) {
					chosenPath.add(elem.destination);
				}
			}
			if (chosenPath.size() > 1)
				chosenPaths.add(chosenPath);
		}

		String result = "";

		for (ArrayList<Integer> path : chosenPaths) {
			if (path == null || path.size() == 0)
				continue;
			String current_path = getRobotString(path);
			result += current_path + "; ";
		}

		//System.out.println(result.substring(0, result.length() - 2));
		return result.substring(0, result.length() - 2);
	}
	
	Point findRobotByIndex(int index, Point[] robots){
		for(Point robot : robots){
			if(robot.index == index){
				return robot;
			}
		}
		return null;
	}

	public double[] findNearestPointInCluster(int current, Cluster cluster) {
		if(cluster.robotsWoken == cluster.totalRobots) return null;

		Point[] robotsInCluster = cluster.getRobotPoints();
		double[] res = new double[2];
		double minDist = MAXVAL;
		int minIndex = robotsInCluster[0].index;
		int minI = 0;
		
		boolean allAwake = true;

		for (int i = 0; i < robotsInCluster.length; i++) {
			if (!robotsInCluster[i].hasWoken) {
				allAwake = false;
				if (minDist > floydTable[current][i]) {
					minDist = floydTable[current][i];
					minIndex = robotsInCluster[i].index;
					minI = i;
				}
			}
		}
		
		if(allAwake){
			return null;
		}
		
		robotsInCluster[minI].hasWoken = true;
		res[0] = minIndex;
		res[1] = minDist;
		return res;
	}

	public Cluster getNearestCluster(Point p1, HashMap<Integer, Cluster> clusters){
		double minDist = MAXVAL;
		Cluster minCluster = null;

		int claimedCount = 0;

		for(Map.Entry<Integer, Cluster> entry : clusters.entrySet()){
			Cluster c = entry.getValue();
			if(c.claimed){
				claimedCount++;
				continue;
			}

			Point p2 = c.centreOfMassPoint;
			if(p2.equals(p1)){
				continue;
			}

			if(minDist > floydTable[p1.index][p2.index]){
				minDist = floydTable[p1.index][p2.index];
				minCluster = c;
			}
		}

		//if all clusters are claimed - return null
		if(claimedCount == clusters.size()){
			//System.out.println("ALL CLAIMED");
			return null;
		}

		return minCluster;
	}

	public HashMap<Integer, Cluster> generateClusters(int k, Point[] robotPoints, double[][] floydTable){
		if(k > robotPoints.length) k = robotPoints.length;

		HashMap<Integer, Cluster> clusters = new HashMap<>();
		//initialise clusters (each robot is a cluster of size 1 initially)
		for(int i = 0; i < robotPoints.length; i++){
			robotPoints[i].cluster = i;
			robotPoints[i].index = i;

			Cluster c = new Cluster(i);
			c.robots.add(robotPoints[i]);
			clusters.put(i, c);
		}

		double[][] floydCopy = new double[robotPoints.length][robotPoints.length];
		System.arraycopy(floydTable, 0, floydCopy, 0, robotPoints.length);

		int numClusters = clusters.size();

		double minDist = MAXVAL;
		int minIndexA = 0;
		int minIndexB = 0;

		while(numClusters > k) {
			minDist = MAXVAL;
			for (int i = 0; i < robotPoints.length; i++) {
				for (int j = 0; j < robotPoints.length; j++) {
					if (floydCopy[i][j] < minDist && i!=j) {
						minDist = floydCopy[i][j];
						minIndexA = i;
						minIndexB = j;
					}
				}
			}
			floydCopy[minIndexA][minIndexB] = MAXVAL;
			floydCopy[minIndexB][minIndexA] = MAXVAL;

			int clusterIndexA = robotPoints[minIndexA].cluster;
			int clusterIndexB = robotPoints[minIndexB].cluster;

			Cluster a = clusters.get(clusterIndexA);
			Cluster b = clusters.get(clusterIndexB);

			//set edges in the same cluster to MAXVAL so it doesn't check again
			for(Point pointA : a.robots){
				for(Point pointB : b.robots){
					floydCopy[pointA.index][pointB.index] = MAXVAL;
					floydCopy[pointB.index][pointA.index] = MAXVAL;
				}
			}

			if(a==null || b==null){
				System.out.println("cluster not found " + clusterIndexA + " " + clusterIndexB);
				continue;
			}

			for(Point p : b.robots){
				//modifies the cluster index for each robot in the union cluster
				robotPoints[p.index].cluster = clusterIndexA;
			}
			a.union(b); //merges b into a
			clusters.remove(clusterIndexB);
			numClusters--;
		}

		for(Map.Entry<Integer, Cluster> entry : clusters.entrySet()){
			Cluster cluster = entry.getValue();

			cluster.calculateCentre();
/*
			System.out.println("Cluster " + cluster.clusterID + " (size: " + cluster.robots.size() + ")");
			for(Point robot : cluster.robots) {
				System.out.print(robot.toString() + " ");
			}
			System.out.println("Centre point: " + cluster.centreOfMassPoint);
			System.out.println("");*/
		}
		/*
		ArrayList<Cluster> finalOutput = new ArrayList<>();
		for(Map.Entry<Integer, Cluster> entry : clusters.entrySet()) {
			Cluster cluster = entry.getValue();
			cluster.calculateCentre();
			finalOutput.add(cluster);
		}*/
		return clusters;
	}
}
