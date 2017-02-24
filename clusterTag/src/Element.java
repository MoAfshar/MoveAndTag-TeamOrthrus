public class Element implements Comparable{
    int robot;
    int destination;
    double totalDistance;
    int robotsAvailable = 2;

    public Element (int robot, int destination, double totalDistance) {
        this.robot = robot;
        this.destination = destination;
        this.totalDistance = totalDistance;
    }

    public void useRobot () {
        this.robotsAvailable--;
    }

	@Override
	public int compareTo(Object e) {
		Element o2=(Element) e;
		return (int)(this.totalDistance - o2.totalDistance);
	}
}