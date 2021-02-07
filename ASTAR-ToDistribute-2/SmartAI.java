import java.awt.Point;
import java.util.Comparator;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;


public class SmartAI implements AIModule
{
  public class Node
  {
    Node parentNode;
    Point coordinates;
    double h;
    double g;
    double f;


    public Node(TerrainMap map, Point coordinates, Node parentNode)
    {
      this.coordinates = coordinates;
      this.parentNode = parentNode;
      if (parentNode == null){
        this.g = 0.0;
        this.h = 0.0;
      }
      else {
        this.g = map.getCost(this.coordinates, map.getEndPoint());
        this.h = getHeuristic(map, this.coordinates);
      }

      this.f = this.g + this.h;
    }
  }

  class newComparator implements Comparator<Node>
  {
    public int compare(Node n1, Node n2)
    {
      if (n1.f > n2.f){
        return 1;
      }
      else {
        return -1;
      }
    }
  }

  public List<Point> createPath(final TerrainMap map)
  {
     Point startPoint = map.getStartPoint();
     Point endPoint = map.getEndPoint();
     List<Node> visitedNodes = new ArrayList<Node>();
     PriorityQueue<Point> unvisitedNodes = new PriorityQueue<>();
     unvisitedNodes.add(map.getStartPoint());

     while (unvisitedNodes.size() > 0){
       Node currentNode = new Node(map, unvisitedNodes.poll(), null);
    //   if (currentNode.coordinates == map.getEndPoint()){
    //     return constructPath(map, currentNode);
    //   }
    //   visitedNodes.add(currentNode);
    //   ArrayList neighborNodes = map.getNeighbors(currentNode.coordinates);
    //   for (Point neighbor: neighborNodes){
    //     Node temp = newNode(map, neighbor, currentNode);
    //     if(!unvisitedNodes.contains(currentNode.coordinates) && !visitedNodes.contains(currentNode.coordinates)){
    //       unvisitedNodes.add(temp);
    //     }
    //     else if (unvisitedNodes.contains(temp.coordinates) && temp.f < unvisitedNodes.get(temp).data.f){
    //       if(temp.f < unvisitedNodes.get(temp).coordinates.f){
    //         unvisitedNodes.remove(temp);
    //         unvisitedNodes.add(temp);
    //       }
    //     }
    //   }
    }
    List<Point> foo = new ArrayList<>();
    foo.add(map.getStartPoint());
    return foo;
  }

//  public List<Point> constructPath(final TerrainMap map, final Node currentPoint)
//  {
//    List<Point> bestPath = new ArrayList<Point>();
//    while (currentPoint.parent != null){
//      bestPath.add(currentNode.coordinates);
//      currentPoint = currentPoint.parentNode;
//    }
//    return bestPath;
//  }

  public double getHeuristic(final TerrainMap map, final Point currentPoint)
  {
    Point endPoint = map.getEndPoint();
    double costStraight = 1.0;
    double costDiagonal = Math.sqrt(2);
    double dX = Math.abs(endPoint.x - currentPoint.x);
    double dY = Math.abs(endPoint.y - currentPoint.y);
    // find how to get height

    double octileDistance = (costStraight * Math.max(dX, dY)) + ((costDiagonal-costStraight)*Math.min(dX, dY));

    return 0.0;
  }
}
