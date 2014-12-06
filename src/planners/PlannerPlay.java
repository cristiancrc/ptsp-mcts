package planners;

import java.util.LinkedList;
import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  TODO:
 *  - does this work well on nodes?
 *  - is an edge approach better / faster / more complete?
 *  @author Cristian
 *  @version 141204
 */

public class PlannerPlay extends Planner {

    @SuppressWarnings("unchecked")
	public PlannerPlay(Game a_gameCopy)
    {
    	System.out.println("***play opt planner***");
        long timeStart = System.currentTimeMillis();
    	m_graph = new Graph(a_gameCopy);
    	
    	verbose = true;
    	
    	//get a greedy plan
		Planner planner = new PlannerGreedy(a_gameCopy);
    	LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();//the list of waypoints
		waypointList = planner.getOrderedWaypoints();//get the planned route
		long timeAfterGreedy = System.currentTimeMillis();
		System.out.println(" Time spent for greedy planner: " + (timeAfterGreedy - timeStart) + " ms.");		

		//add ship position as waypoint
    	Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);        
    	m_orderedWaypoints.add(wpShip);
    	distanceMatrix = createDistanceMatrix(waypointList);            	
		long timeAfterMatrix = System.currentTimeMillis();
		System.out.println(" Time spent to build distance matrix: " + (timeAfterMatrix - timeAfterGreedy) + " ms.");
		
		//build paths, based on the greedy result
    	double pathMinCost = getPathDistance(waypointList);//result from greedy	
		double pathCost = 0;//local search result
		LinkedList<Waypoint> aPath = new LinkedList<>();//stores built paths
		LinkedList<Waypoint> aPathRev = new LinkedList<>();//stores built paths
		
		int tempLimit = waypointList.size();
		tempLimit = 6;
        LinkedList<Integer> intPath = new LinkedList<>();
		LinkedList<Integer> intPathRev = new LinkedList<>();

		
		
		showList(waypointList);
		for (int i = 0; i < tempLimit; i++)
		{
			System.out.print(" " + i);
		}
		System.out.println("");
		
		
		
		
		
		
		long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent searching: " + (timeAfter - timeAfterMatrix) + " ms.");    	
		System.out.println("Path distance:" + getPathDistance(m_orderedWaypoints));			
		System.out.println("3Opt Planner time: " + (timeAfter - timeStart) + " ms.");	
		System.exit(0);
    }
}
