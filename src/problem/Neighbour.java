package problem;
//import static problem.Util.*;

import java.awt.geom.Point2D;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;

public class Neighbour {
  public Point2D pos;
  public GridType type;

  public Neighbour(Point2D pos, GridType type) {
    this.pos = pos;
    this.type = type;
  }

  public Boolean isInList(List<GridNode> list) {
      for(GridNode g : list) {
        if(Util.equalPositions(g.pos, this.pos)) return true;
      }
      return false;
  }

  public GridNode isInPrioQueue(PriorityQueue<GridNode> queue) {
    Iterator iter = queue.iterator();
    while(iter.hasNext()) {
      GridNode current = (GridNode)iter.next();
      if(Util.equalPositions(current.pos, this.pos)) {
        return current;
      }
    }
    return null;
  }

}
