package planners;

import java.util.LinkedList;

import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;

/**
 *  greedy planner
 *  goes through the list waypoints and gets the closest one
 *  creates just the distance matrix if it isn't present
 *  @author Cristian
 *  @version 141128
 */
public class PlannerGreedy extends Planner {
		
	public PlannerGreedy(Game a_gameCopy)
	{
    	System.out.println("***greedy planner***");
        long timeStart = System.currentTimeMillis();
    	m_graph = new Graph(a_gameCopy);
    	@SuppressWarnings("unchecked")
		LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();//a list of all waypoints 

    	//add ship position as waypoint
        Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);    	
    	if(distanceMatrix.size() == 0)
    	{
    		waypointList.addFirst(wpShip);
    		distanceMatrix = createDistanceMatrix(waypointList);
    		waypointList.removeFirst();
    	}    	    	
    	m_orderedWaypoints.addFirst(wpShip);
    	
		//iterate through the other waypoints to find the closest one to each next one
		int checkedWaypoints = 0;
		Waypoint wpFrom = wpShip;		
    	double distanceTravelled = 0;//total distance, no distance matrix needed	
		while(waypointList.size() > 0 )
		{
			if(verbose) System.out.println("\n" + checkedWaypoints++ + ": wpFrom: " + wpFrom.getName() + ", wpList size " + waypointList.size());// + ", ordered " + orderedWaypoints.size());
			
			Waypoint closestWaypoint = null;
			double minPathCost = Double.MAX_VALUE;//path between the two waypoints we are checking
	    	
			for(Waypoint wpEnd : waypointList)
			{
				if(verbose) System.out.print("...checking " + wpEnd.getName());				
				double distance = distanceMatrix.get(wpFrom).get(wpEnd);		
				if(distance < minPathCost)
				{
					closestWaypoint = wpEnd;
					minPathCost = distance;
				}			
			}			
			if(verbose) System.out.println("closest is " + closestWaypoint.getName());
			distanceTravelled += minPathCost;
			m_orderedWaypoints.add(closestWaypoint);
			wpFrom = closestWaypoint;//next search starting from this node
			waypointList.remove(closestWaypoint);//remove node from list
		}		
		long timeAfter = System.currentTimeMillis();
		System.out.println("Path distance:" + distanceTravelled);
		System.out.println("Greedy Planner running time: " + (timeAfter - timeStart) + " ms.");
    }	
}
