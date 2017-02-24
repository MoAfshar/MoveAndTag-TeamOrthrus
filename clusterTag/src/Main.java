import java.util.ArrayList;

public class Main {

    public static void main(String[] args){
        Parser parser = new Parser("C:/Users/pnash/Desktop/ParserTag/clusterTag/src/robots.mat");

        ArrayList<ArrayList<Point>> allInput = parser.mapsOfPoints;
        ArrayList<ArrayList<Point>> allRobots = parser.mapsOfRobots;
        ArrayList<ArrayList<Point[]>> allObstacles = parser.mapsOfObstacles;


		//solve problem
    	ArrayList<String> lines = new ArrayList<String>();

    	lines.add("orthrus");
    	lines.add("4g7f4crvlfh6sjmkj42255fmjv");

    	for(int i = 0; i < allInput.size(); i++){

    		Algorithm alg = new Algorithm();

    		ArrayList<Point> pointsList = allInput.get(i);
    		Point[] allPoints = parser.convertToArr(pointsList);

            alg.init(allPoints, allObstacles.get(i), allPoints.length, i);

            String output = (i+1) + ": ";

            String solution;
            if (allRobots.get(i).size()==2)  {
                solution = alg.visitAll(allPoints, allRobots.get(i).size());
                solution = solution.substring(0, solution.length()-2);
            }
            else {
                solution = alg.callBuildTree(allPoints, allRobots.get(i).size());
            }

            output = output + solution;


            lines.add(output);
            System.out.println(output);
    	}

    	parser.writeOutputFile(lines, "output.txt");

    }



}
