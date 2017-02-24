import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;

public class Parser {
    public ArrayList<ArrayList<Point>> mapsOfRobots = new ArrayList<>();
    public ArrayList<ArrayList<Point[]>> mapsOfObstacles = new ArrayList<>();
    public ArrayList<ArrayList<Point>> mapsOfPoints = new ArrayList<>();
    public static int situations = 30;

    //to make the code look cleaner
    public static Point[] convertToArr(ArrayList<Point> list){
        return list.toArray(new Point[list.size()]);
    }

    public Parser(String filename) {
        ArrayList<String> maps = getMapsFromFile(filename, situations);
        for (int i = 0; i < situations; i++) {
            String toProcess = maps.get(i);
            //splits the points and robots as separated by the hash
            String[] splitRobotsAndObstacles = toProcess.split("#");
            //splits up each individual robot
            String[] stripRobots = splitRobotsAndObstacles[0].split(":");
            String[] obstaclesWithSemicolons = {};
            if (splitRobotsAndObstacles.length > 1) {
                obstaclesWithSemicolons = splitRobotsAndObstacles[1].split(";");
            }

            //adds all robots to the maps
            ArrayList<Point> robotPositions = getRobotPositions(stripRobots);
            ArrayList<Point[]> obstaclePositions = getObstaclePositions(obstaclesWithSemicolons);
            mapsOfRobots.add(robotPositions);
            mapsOfObstacles.add(obstaclePositions);

            //combines them into a total point list as well
            mapsOfPoints.add(combinePoints(robotPositions, obstaclePositions));

        }
    }

    private ArrayList<Point> combinePoints(ArrayList<Point> robotPositions, ArrayList<Point[]> obstaclePositions) {
        ArrayList<Point> combined = new ArrayList<>();
        for (Point p : robotPositions) {
            combined.add(p);
        }
        for (Point[] o : obstaclePositions) {
            for (Point point : o) {
                combined.add(point);
            }
        }
        return combined;
    }

    public ArrayList<Point> getRobots(int mapNum) {
        return mapsOfRobots.get(mapNum);
    }

    public ArrayList<Point[]> getObstacles(int mapNum) {
        return mapsOfObstacles.get(mapNum);
    }

    private ArrayList<Point> getRobotPositions(String[] stripRobots) {
        //a side effect of the following line is that the number and the colon are also removed
        String RobotPositions = stripRobots[1].replaceAll("[\\[\\](){}]","");
        String RobotPositionsStripped = RobotPositions.replaceAll("\\s+","");
        String[] data = RobotPositionsStripped.split(",");

        ArrayList<Point> robotPositions = new ArrayList<>();
        for (int i = 0; i < data.length; i+=2) {
            robotPositions.add(new Point(Double.parseDouble(data[i]), Double.parseDouble(data[i+1])));
        }

        return robotPositions;
    }

    private ArrayList<Point[]> getObstaclePositions(String[] obstaclesWithSemicolons) {
        ArrayList<Point[]> obstacles = new ArrayList<>();
        for (String s: obstaclesWithSemicolons) {
            obstacles.add(obstacleToPoints(s));
        }
        return obstacles;
    }

    private Point[] obstacleToPoints(String obstacles) {
        String obstaclePositions = obstacles.replaceAll("[\\[\\](){}]","").replaceAll("\\s+","");
        String[] data = obstaclePositions.split(",");

        ArrayList<Point> obstaclePointList = new ArrayList<>();
        for (int i =0; i < data.length; i+=2) {
            obstaclePointList.add(new Point(Double.parseDouble(data[i]), Double.parseDouble(data[i+1])));
        }

        return obstaclePointList.toArray(new Point[0]);
    }

    private ArrayList<String> getMapsFromFile(String path, int situations) {
        ArrayList<String> lines = new ArrayList<>();

        try {
            BufferedReader inFile = new BufferedReader(new FileReader(path));
            for (int i = 0; i < situations; i ++) {
                lines.add(inFile.readLine());
            }
        }
        catch (Exception e) {
            System.out.println("lol error in file IO");
        }


        return lines;
    }

    public static void writeOutputFile(ArrayList<String> lines, String destPath){
        PrintWriter pw = null;
        try {
            pw = new PrintWriter(new FileWriter(destPath));
        } catch (Exception e) {
            e.printStackTrace();
        }

        for(int i = 0; i < lines.size(); i++){
            pw.write(lines.get(i));
            pw.println();
        }

        pw.close();
    }

    public static ArrayList<Point[]> readOutputFile(String filePath) {
        ArrayList<Point[]> allPoints = new ArrayList<Point[]>();

        String line;

        try {

            BufferedReader reader = new BufferedReader(new FileReader(filePath));
            reader.readLine();
            reader.readLine();
            while ((line = reader.readLine()) != null) {
                line = line.replaceAll("\\s", ""); // remove all whitespaces
                line = line.split(":")[1];
                String[] pointStr = line.split(",");

                Point[] points = new Point[pointStr.length / 2];
                int index = 0;

                for (int i = 0; i < pointStr.length - 1; i += 2) {
                    pointStr[i] = pointStr[i].replaceAll("[()]", "");
                    pointStr[i + 1] = pointStr[i + 1].replaceAll("[()]", "");
                    double x = Double.parseDouble(pointStr[i]);
                    double y = Double.parseDouble(pointStr[i + 1]);
                    points[index++] = new Point(x, y, true);
                }

                allPoints.add(points);
            }
            reader.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        return allPoints;
    }
}

