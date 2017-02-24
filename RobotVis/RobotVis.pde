import java.util.*;
import java.io.FileReader;
import java.io.*;

class Point{
  Double x;
  Double y;
  
  public Point(Double x, Double y){
    this.x = x;
    this.y = y;
  }
  
 public void setX (Double x){
   this.x = x;
 }
 
 public void setY (Double y){
   this.y = y;
 } 
 
 public Double getX(){
   return x;
 }
 
 public Double getY(){
   return y;
 }
  
}

class Colour {
  float r = 0;
  float g = 0;
  float b = 0;
  Colour(float r, float g, float b) {
    this.r = r;
    this.g = g;
    this.b = b;
  }
}

List<List<Point>> pathList;
List<Point> robotPositions;
List<List<Point>> obstacles;
List<Colour> colours = new ArrayList();

int translateX = 0;
int translateY = 0;
float scale = 10;

int xOffset = 0;
int yOffset = 0;



void setup(){
  int whichMap = 28;
  
  //reads input files into a list
  List<String> lines = getMapsFromFile("C:/Users/Mo/Desktop/robots.txt", 30);
  
  //processes one input fully
  String sampleString = lines.get(whichMap-1);  
  String[] splitRobotsAndObstacles = sampleString.split("#");  
  String[] stripRobots = splitRobotsAndObstacles[0].split(":");  
  String[] obstaclesWithSemicolons = {};
  if (splitRobotsAndObstacles.length > 1) {
      obstaclesWithSemicolons = splitRobotsAndObstacles[1].split(";");    
  }
    


  robotPositions = getRobotPositions(stripRobots);
  //TODO: list of obstacles here as above for robots
  obstacles = getObstaclePositions(obstaclesWithSemicolons);
  
  
  
  //reads output files into a list
  List<String> outputLines = getMapsFromFile("C:/Users/Mo/Desktop/backup_output_30.txt",32);
  
  //processes one output file fully
  String outputString = outputLines.get(whichMap+1);
  pathList = createPathList(outputString);

  //System.out.println("ROBOT POSITIONS: " );
  //for (Point p : robotPositions) {
  //  System.out.println(p.getX() + "," + p.getY());
  //}
  
  //System.out.println("OBSTACLE POSITIONS: ");
  //for (List<Point> l : obstacles) {
  //  for (Point p: l) {
  //    System.out.print("(" + p.x + "," + p.y + ")" + " ");
  //  }
  //  System.out.println();
  //}

  size(1000,1000);
  generateColours(robotPositions.size());
}

void draw() {
  background(50);
  noFill(); 
  fill(255, 255, 255);
  textSize(32);
  text("Orthrus - Computational Geometry Simulation", 10, 30); 
  translate(500+translateX, 500+translateY);
  scale(scale);
  strokeWeight(0.2);
  processInput();

  drawObstacles(obstacles);
  drawRobots(robotPositions);
  drawPaths(pathList);
}

void generateColours(int number) {
  for (int i = 0; i < number; i++) {
    colours.add(new Colour(random(0,255), random(0,255), random(0,255)));
  }
}

void drawRobots(List<Point> robots){
  strokeWeight(0.05);
  fill(255,0,0);
  for (Point p: robots) {
    float size = 0.2;
    ellipse(p.getX().floatValue(), p.getY().floatValue(), size, size);
    fill(0,255,255);
  }
}

void drawObstacles(List<List<Point>> obstacleList) {
  for (List<Point> l : obstacleList) {
    drawPolygon(l);
  }
  
}

void drawPolygon(List<Point> pointList){
    stroke(0);
    fill(204,102,0);
    beginShape();
    for (Point p: pointList) {
      vertex(p.getX().floatValue(), p.getY().floatValue());
    }
    endShape(CLOSE);
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scale -= e * 1.5;
}

void mouseDragged() 
{
  translateX = mouseX-xOffset; 
  translateY = mouseY-yOffset; 
}

void mousePressed() {
  xOffset = mouseX-translateX; 
  yOffset = mouseY-translateY; 

}

void processInput() {
  int translateSpeed = 5;
  float scaleSpeed = 0.5;
   if (keyAvailable())
    {
      switch(key)
      {
        case 'a':
          translateX -=translateSpeed;
          break;
        case 's':
          translateY +=translateSpeed;
          break;
        case 'd':
          translateX +=translateSpeed;
          break;
        case 'w':
          translateY -=translateSpeed;
          break;
        case 'u':
          scale +=scaleSpeed;
          break;
        case 'j':
          scale -=scaleSpeed;
          break;
      }
    }
}

boolean keyAvailable() {
  return (key=='a' || key=='w' || key=='s' || key=='d' || key=='u' || key=='j') && keyPressed;
}

List<String> getMapsFromFile(String path, int situations) {
    //read input file
  List<String> lines = new ArrayList();
  
    try {
        BufferedReader inFile = new BufferedReader(new FileReader(path));
        for (int i = 0; i < situations; i++) {
            lines.add(inFile.readLine());
        }
      inFile.close();
      return lines;
      }
      catch (Exception e) {
        return new ArrayList<String>();
      }
      
}


List<List<Point>> createPathList(String outputString) {
  String removeNumber = outputString.split(":")[1];
  String[] robotPaths = removeNumber.split(";");
  
  List<List<Point>> pathList = new ArrayList();
  for (int i = 0; i < robotPaths.length; i++) {
    String specificPath = robotPaths[i].replaceAll("[\\[\\](){}]","").replaceAll("\\s+","");
    String[] data = specificPath.split(",");
    
    List<Point> path = new ArrayList();
    for (int j =0; j < data.length; j+=2) {
      path.add(new Point(Double.parseDouble(data[j]), Double.parseDouble(data[j+1])));
    }
    pathList.add(path);
  }
  return pathList;
}

void drawPaths(List<List<Point>> pathList){
 strokeWeight(0.05);
 //stroke(255,0,0);
 for (int i = 0; i < pathList.size(); i++) {
   List<Point> currentRobot = pathList.get(i);
   Colour c = colours.get(i);
   stroke(c.r , c.g, c.b);
     for (int j = 0; j < currentRobot.size() - 1; j++) {
       Point startRobot = currentRobot.get(j);
       Point nextRobot = currentRobot.get(j+1);
       line(startRobot.x.floatValue(), startRobot.y.floatValue(), nextRobot.x.floatValue(), nextRobot.y.floatValue());
     }
  }
 }

List<List<Point>> getObstaclePositions(String[] obstaclesWithSemicolons) {
  List<List<Point>> obstacles = new ArrayList();
  for (String s: obstaclesWithSemicolons) {
    obstacles.add(obstacleToPoints(s)); 
  }
  return obstacles;
}

List<Point> obstacleToPoints(String obstacles) {
  String obstaclePositions = obstacles.replaceAll("[\\[\\](){}]","").replaceAll("\\s+","");
  String[] data = obstaclePositions.split(",");
  
  List<Point> obstaclePointList = new ArrayList();
  for (int i =0; i < data.length; i+=2) {
    obstaclePointList.add(new Point(Double.parseDouble(data[i]), Double.parseDouble(data[i+1])));
  }
  
  return obstaclePointList;
}

List<Point> getRobotPositions(String[] stripRobots) {
  //a side effect of the following line is that the number and the colon are also removed
  String RobotPositions = stripRobots[1].replaceAll("[\\[\\](){}]","");
  String RobotPositionsStripped = RobotPositions.replaceAll("\\s+","");
  String[] data = RobotPositionsStripped.split(",");
  
  List<Point> robotPositions = new ArrayList();
  for (int i = 0; i < data.length; i+=2) {
    robotPositions.add(new Point(Double.parseDouble(data[i]), Double.parseDouble(data[i+1])));
  }
  
  return robotPositions;
}