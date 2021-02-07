import java.util.List;
import java.awt.Point;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Set;
import java.util.HashSet;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.Collections;

// g =  the movement cost to move from the starting
//      point to a given square on the grid, following the path generated to get there
// h =  the estimated movement cost to move from that given square
//       on the grid to the final destination.

public class PerfectAI implements AIModule {
     static class PointTile implements Comparable<PointTile>{
            Point point;
            PointTile parent;

            Double h;
            Double g;
            Double f;

           public PointTile(Point point, PointTile parent, TerrainMap map) {
                this.point = point;
                this.parent = parent;

                //in case parent is null
                if (parent == null) {
                    this.h = 0.0;
                    this.g = 0.0;
                } else {
                    // Set h(n') to be the heuristically estimate distance to node_goal
                    this.h = getHeuristic(this, map.getEndPoint(), map);

                    double costFromParent = map.getCost(parent.point, point);

                    // Set g(n') to be g(n) plus the cost to get to n' from n
                    this.g = parent.g + costFromParent;
                }
                // Set f(n') to be g(n') plus h(n')
                this.f = this.h + this.g;
            }

            public int compareTo(PointTile o) {
                int fDiff = this.f.compareTo(o.f);
                if (fDiff == 0) {
                    int xDiff = this.point.x - o.point.x;
                        if (xDiff == 0) {
                            int yDiff = this.point.y - o.point.y;
                                if (yDiff == 0) {
                                    //same point and same value of f - they are the same
                                    return 0;
                                } else {
                                    return yDiff;
                                }
                        } else {
                            return xDiff;
                        }
                } else {
                    return fDiff;
                }
            }

            public String toString(){
                return this.point.toString() + " g:"+this.g+" h:"+this.h+" f:"+this.f;
            }

            public boolean equals(Object o) {
                if (o instanceof PointTile) {
                    return this.point.equals(((PointTile) o).point);
                }
                return false;
            }

            public int hashCode() {
                return this.point.hashCode();
            }

         }//end of subclass PointTile

    private static int findGCD(int number1, int number2) {
        //base case
        if(number2 == 0){
            return number1;
        }
        return findGCD(number2, number1%number2);
    }

    public static double euclideanDistance (Point p1, Point p2) {
        return Math.sqrt( Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2) );
    }

  public static double diagonalDistance (Point p1, Point p2) {
    int D = 1;
    int D2 = 1;
    int dx = Math.abs(p1.x - p2.x);
    int dy = Math.abs(p1.y - p2.y);
    return (double) (D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy));
  }
  
  /**
   * If h(n) is always lower than (or equal to) the cost of moving from n to the goal, then A* is 
   * guaranteed to find a shortest path. The lower h(n) is, the more node A* expands, making it slower.
   */
  public static double getHeuristic (PointTile currentPoint, Point endPoint, TerrainMap map) {
      
        double euclideanDist = euclideanDistance(currentPoint.point, endPoint);

        //100 x 100
        //PathCost, 98.75, Uncovered, 4796, TimeTaken, 179

        //500 x 500
        //PathCost, 416.0625, Uncovered, 48242, TimeTaken, 890

        //heuristic dealing with keeping the line straight

        //If we move the end point to the origin then
        //vector A = (a1, a2) and B = (b1, b2), the cosine similarity is given as:
        //(a1 b1 + a2 b2) / sqrt(a1^2 + a2^2) sqrt(b1^2 + b2^2)

        Point startPoint = map.getStartPoint();
        Point vectorA    = new Point(endPoint.x - startPoint.x, endPoint.y - startPoint.y);
        Point vectorB    = new Point(endPoint.x - currentPoint.point.x, endPoint.y - currentPoint.point.y);

        //dot product returns one number
        double dotProduct  = (vectorA.x * vectorB.x) + (vectorA.y * vectorB.y);
        double prodLengths =
          Math.sqrt(Math.pow(vectorA.x,2) + Math.pow(vectorA.y,2)) *
          Math.sqrt(Math.pow(vectorB.x,2) + Math.pow(vectorB.y,2));

        double cosSim = 1.0;

        if (prodLengths != 0) {
          cosSim = dotProduct / prodLengths;
        }

        double angularCosDist = 1.0;

        if (cosSim != 0.0) {
          angularCosDist = (1 * Math.acos(cosSim)) / 3.14;
        }

        double p = angularCosDist;

	    //euclidean distance seems to not provide the optimal distance
	    //it seems to be wrong close to the destination
        double heuristic = euclideanDist;
	
        //multiply h by the normalization value p
        //comment out this line to make it work just like euclidean
	    heuristic *= 1.0 + p;

	    //euclidean distance seems to lead to the wrong path
	    //once closer to end point. Threshold and reduce once we get close
	    if (euclideanDist < 50.0) {
	        heuristic = 0;
	    }

	//heuristic = 0.0;
	

	/**
	   latentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 1 -load MTAFT.XYZ 
	   PerfectAI
	   PathCost, 550.7718270299947, Uncovered, 1092515, TimeTaken, 16986

	*/
	
	//System.out.println("Heuristic: "+heuristic + " max="+maxEuclideanDistance+" diff="+distDiff+" dist="+euclideanDist+" p="+p);

	
	/**
	   Dijkstra

	   latentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 1
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 416.0625, Uncovered, 231907, TimeTaken, 2908
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 2
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 404.34375, Uncovered, 232197, TimeTaken, 2829
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 3
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 396.15625, Uncovered, 229329, TimeTaken, 2853
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 4
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 419.453125, Uncovered, 220874, TimeTaken, 2739
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 5
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 378.875, Uncovered, 224127, TimeTaken, 2789

	   Euclidean Distance

	   latentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 1
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 416.0625, Uncovered, 46126, TimeTaken, 843
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 2
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 404.34375, Uncovered, 50800, TimeTaken, 773
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 3
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 396.15625, Uncovered, 44651, TimeTaken, 852
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 4
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 421.53125, Uncovered, 41160, TimeTaken, 668
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 5
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 379.09375, Uncovered, 38511, TimeTaken, 810

	   cossim

	   latentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 1
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 416.0625, Uncovered, 37188, TimeTaken, 728
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 2
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 404.34375, Uncovered, 39322, TimeTaken, 570
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 3
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 396.65625, Uncovered, 37450, TimeTaken, 698
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 4
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 424.0390625, Uncovered, 31156, TimeTaken, 535
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 5
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 379.28125, Uncovered, 31550, TimeTaken, 599

	   cossim + length threshold

	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 1
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 416.0625, Uncovered, 37586, TimeTaken, 725
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 2
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 404.34375, Uncovered, 40546, TimeTaken, 682
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 3
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 396.15625, Uncovered, 37114, TimeTaken, 775
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 4
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 419.453125, Uncovered, 27424, TimeTaken, 502
	   ^Clatentspace:ASTAR-ToDistribute-2$ java Main PerfectAI -seed 5
	   Done: same-f[0]
	   PerfectAI
	   PathCost, 379.28125, Uncovered, 29817, TimeTaken, 588
	   
	*/

	

        return heuristic;
    
  }
   
    public List<Point> createPath(final TerrainMap map) {
        //pointTiles, can not get a specific thing from a set, so we need a map
        //that maps the point to the pointitle so we can always go and get that
        Map<Point, PointTile> pointTiles = new HashMap<>();
        SortedSet<PointTile> open = new TreeSet<>();

        PointTile startingPointTile = new PointTile(map.getStartPoint(), null, map);
        pointTiles.put(startingPointTile.point, startingPointTile);
        open.add(startingPointTile);

        PointTile end = null;
        Point endPoint = map.getEndPoint();
        int numSameF = 0;

        while(open.size() > 0) {

            //get the next best point and remove it from open
            PointTile q = open.first();
            open.remove(q);

            if (q.point.equals(endPoint)) {
                end = q;
                break;
            } else {
                Point[] neighbors = map.getNeighbors(q.point);
                for (Point neighbor: neighbors) {

                    //create a new tile for this point
                    PointTile neighborTile = new PointTile(neighbor, q, map);

                    //if this neighbor already exists - compare this new one to the existing one
                    if (pointTiles.containsKey(neighbor)) {

                        PointTile compareNeighborTile = pointTiles.get(neighbor);

                        if (compareNeighborTile.f <= neighborTile.f) {
                            //skip neighbor, because we have something in open that is as good or better
                            continue;
                        }

                        //if we got to here then the new tile at this position is better so
                        //remove the old from open
                        open.remove(compareNeighborTile);

                        //need to remove this child from the parent

                    } else {
                        //this is brand new point that we had not yet visited
                    }

                    //add this new one to open and overwrite the tile in the map
                    pointTiles.put(neighbor, neighborTile);
                    open.add(neighborTile);

                }//end of for loop
            }//end of else loop
        }//end of while loop

        List<Point> optimumPath = new ArrayList<>();

        while (end != null) {
            optimumPath.add(end.point);
            end = end.parent;
        }

        Collections.reverse(optimumPath);

        return optimumPath;

    }// end of method: createPath

    public static void main(String[] args) {
      SortedSet<PointTile> open = new TreeSet<PointTile>();
      
      PointTile p1 = new PointTile(new Point(10, 2), null, null);
      p1.f = 0.4;
      open.add(p1);
      
      PointTile p2 = new PointTile(new Point(10, 3), null, null);
      p2.f = 0.5;
      open.add(p2);
      
      while (open.size() > 0) {
	    PointTile pt = open.first();
	    System.out.println(pt);
	    open.remove(pt);
      }
    }
}
