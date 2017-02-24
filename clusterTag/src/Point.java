import java.util.ArrayList;

public class Point {
	//x and y position of the point
    public double x;
    public double y;
    
    //a list of points that are visible from the current point
    //(no obstacles in the line of path)
    public ArrayList<Point> visible;
    
    //a point can be a robot or a obstacle point
    public boolean isRobot = false;
    
    //only applies to robots - can be either awake or sleeping
    public boolean hasWoken = false;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
        this.visible = new ArrayList<Point>();
    }

    public boolean isEqual (Point other) {
        return ((this.x==other.x)&&(this.y==other.y));
    }

    public boolean isVisible (Point p) {
        return visible.contains(p);
    }

    public Point(double x, double y, boolean isRobot){
        this.x = x;
        this.y = y;
        this.visible = new ArrayList<Point>();
        this.isRobot = isRobot;
    }

    public void setVisible(ArrayList<Point> visible) {
        this.visible = visible;
    }

    public String toString(){
        return "(" + x + ", " + y + ")";
    }
}
