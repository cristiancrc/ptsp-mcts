package planners;

import java.util.LinkedList;

import framework.core.Game;
import framework.core.GameObject;
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
		
	public PlannerGreedy(Game aGameCopy)
	{
    	System.out.println("***greedy planner distance***");
    	this.aGameCopy = aGameCopy;
    	this.verbose = false;
	}
	
	public void runPlanner()
	{
        long timeStart = System.currentTimeMillis();
    	aGraph = new Graph(aGameCopy);
    	@SuppressWarnings("unchecked")
		LinkedList<GameObject> waypointList = (LinkedList<GameObject>) aGameCopy.getWaypoints().clone();//a list of all waypoints 
    	    
    	if(distanceMatrix.size() == 0)
    	{
    		createMatrices();
    		//create just the distance matrix
//    		waypointList.addFirst(wpShip);
//    		distanceMatrix = createDistanceMatrix(waypointList);    		
//    		waypointList.removeFirst();
    	}
    	
    	//add ship position as waypoint
        Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);    	
    	orderedWaypoints.addFirst(wpShip);
    	
		//iterate through the other waypoints to find the closest one to each next one
		int checkedWaypoints = 0;
		GameObject wpFrom = wpShip;	
    	double distanceTravelled = 0;//total distance, no distance matrix needed	
		while(waypointList.size() > 0 )
		{
			if(verbose) System.out.println("\n" + checkedWaypoints++ + ": wpFrom: " + wpFrom.getName() + ", wpList size " + waypointList.size());// + ", ordered " + orderedWaypoints.size());
			
			GameObject closestWaypoint = null;
			double minPathCost = Double.MAX_VALUE;//path between the two waypoints we are checking
	    	
			for(GameObject wpEnd : waypointList)
			{
				LinkedList<GameObject> shortList = new LinkedList<>();
				shortList.add(wpFrom);
				shortList.add(wpEnd);
				if(verbose) System.out.print("...checking " + wpEnd.getName());
				double distance = distanceMatrix.get(wpFrom).get(wpEnd);		
				if(distance < minPathCost)
				{
					closestWaypoint = wpEnd;
					minPathCost = distance;
				}			
			}			
			if(verbose) System.out.println(" ::: closest is " + closestWaypoint.getName());
			distanceTravelled += minPathCost;
			orderedWaypoints.add(closestWaypoint);
			wpFrom = closestWaypoint;//next search starting from this node
			waypointList.remove(closestWaypoint);//remove node from list
		}		
		long timeAfter = System.currentTimeMillis();
		System.out.println("Path cost:" + getPathCost(orderedWaypoints));
		System.out.println("Path distance:" + distanceTravelled);
		System.out.println("Greedy Planner running time: " + (timeAfter - timeStart) + " ms.");
    }	
}
