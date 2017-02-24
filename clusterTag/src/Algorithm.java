import java.util.ArrayList;
import java.util.PriorityQueue;

public class Algorithm {
	// global variables
	private double[][] floydTable;
	private int[][] warshalls;
	private double epsilon = 0.000000001;

	private Point[] allPoints;
	private boolean allFound = false;
	private static final double MAXVAL = Double.MAX_VALUE / 2 - 1;
	private ArrayList<ArrayList<Point>> allPaths = new ArrayList<>();

	private int totalRobots;
	private int robotsFound = 1;

	private double calculateDistance(Point p1, Point p2) {
		return Math.sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y)
				* (p2.y - p1.y));
	}

	public void algorithmStart() {

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
		if (p1.isEqual(p3) || p1.isEqual(p4)) {
			return Intersection.P1SAME;
		}

		if (p2.isEqual(p3) || p2.isEqual(p4)) {
			return Intersection.P2SAME;
		}

		if (lineTouchesOrCrossesLine(p1, p2, p3, p4)
				&& lineTouchesOrCrossesLine(p3, p4, p1, p2)) {
			return Intersection.YES;
		}

		return Intersection.NO;
	}

	public void calculatePointVisibility(Point[] allPoints, ArrayList<Point[]> obstaclePoints) {
		int len = allPoints.length;
		ArrayList<Point> visiblePoints = new ArrayList<>();
		boolean collides = false;
		for (int i = 0; i < len; i++) {
			for (int j = 0; j < len; j++) {
				if (j != i) {
					for (Point[] obstaclePoint : obstaclePoints) {
						if (pathCollidesObstacle(allPoints[i], allPoints[j], obstaclePoint)) {
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

	public void init(Point[] allPoints, ArrayList<Point[]> obstaclePoints, int pointNum, int mapNum) {
		calculatePointVisibility(allPoints, obstaclePoints);

		this.allPoints = new Point[pointNum];

		floydTable = new double[pointNum][pointNum];
		warshalls = new int[pointNum][pointNum];
		for (int i = 0; i < pointNum; i++) {
			for (int j = 0; j < pointNum; j++) {
				warshalls[i][j] = -1;
			}
		}

		calcFloydDistances(allPoints);
		calcFloydPaths(allPoints);
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




	/*
	FLOYD WARSHALL STUFF ALL ABOVE, IGNORE WHAT'S BELOW HERE
	 */

	public String path(int x, int y, Point[] points) {
		int k = warshalls[x][y];
		if (k == -1)
			return "";
		if (k == -2)
			return points[y].toString() + ", ";
		if (k == -3)
			return "";
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
		PriorityQueue<Element> elements = new PriorityQueue<Element>() {
			public int compare(Element o1, Element o2) {
				return (int) (o1.totalDistance - o2.totalDistance);
			}
		};
		double[] nearestArray = findNearestPoint(0);
		int nearest = (int) nearestArray[0];
		double dist = nearestArray[1];
		Element first_elem = new Element(0, nearest, dist);
		elements.add(first_elem);
		ArrayList<Element> savedElements = new ArrayList<>();

		while (!elements.isEmpty()) {
			// if (savedElements.size()==numRobots) break;
			// System.out.println("");
			Element current = elements.poll();
			savedElements.add(current);
			if (robotsFound == totalRobots) {
				continue;
			}
			double[] nearest1 = findNearestPoint(current.robot);
			int nearest1_index = (int) nearest1[0];
			if (nearest1_index != current.destination) {
				double nearest1_dist = nearest1[1];
				elements.add(new Element(current.robot, nearest1_index,
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
				elements.add(new Element(current.destination, nearest2_index,
						current.totalDistance + nearest2_dist));
			}
			robotsFound++;
		}
		ArrayList<ArrayList<Integer>> chosenPaths = new ArrayList<ArrayList<Integer>>(
				totalRobots);
		for (int i = 0; i < totalRobots; i++) {
			ArrayList<Integer> chosenPath = new ArrayList<>();
			chosenPath.add(i);
			int elemSize = savedElements.size();
			for (int j = 0; j < elemSize; j++) {
				Element elem = savedElements.get(j);
				if (elem.robot == i) {
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
			return "";
		if (k == -2)
			return allPoints[y].toString() + ", ";
		if (k == -3)
			return "";
		return path_new(x, k) + path_new(k, y);
	}

	public String getRobotString(ArrayList<Integer> pointsIndex) {
		int len = pointsIndex.size();
		String output = allPoints[pointsIndex.get(0)].toString() + ", ";
		for (int i = 1; i < len; i++) {
			output = output
					+ path_new(pointsIndex.get(i - 1), pointsIndex.get(i));
		}
		return output.substring(0, output.length() - 2);
	}
}
