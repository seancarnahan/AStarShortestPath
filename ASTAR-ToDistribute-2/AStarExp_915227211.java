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

public class AStarExp_915227211 implements AIModule {
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


    public static double euclideanDistance (Point p1, Point p2) {
        return Math.sqrt( Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2) );
    }

    /**
     * If h(n) is always lower than (or equal to) the cost of moving from n to the goal, then A* is
     * guaranteed to find a shortest path. The lower h(n) is, the more node A* expands, making it slower.
     */
    public static double getHeuristic (PointTile currentPoint, Point endPoint, TerrainMap map) {
        double euclideanDist = euclideanDistance(currentPoint.point, endPoint);
        double heuristic = euclideanDist;

        double minDistance = 7.62939453125E-6;

        heuristic *= minDistance;

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