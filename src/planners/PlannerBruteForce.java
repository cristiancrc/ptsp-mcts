package planners;

import java.util.LinkedList;
import java.util.Random;

import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  brute force search
 *  will go over the allowed time
 *  @author Cristian
 *  @version 141201
 */

public class PlannerBruteForce extends Planner {

	Random rand = new Random();
	double pathCostMin = Double.MAX_VALUE;
	
    public PlannerBruteForce(Game aGameCopy)
    {
    	System.out.println("***brute force search planner***");
    	this.aGameCopy = aGameCopy;
    }
    
    public void runPlanner()
    {
        long timeStart = System.currentTimeMillis();    	
    	aGraph = new Graph(aGameCopy);

    	//add ship position as waypoint for distance matrix
    	@SuppressWarnings("unchecked")
		LinkedList<GameObject> waypointList = (LinkedList<GameObject>) aGameCopy.getWaypoints().clone();    	    	
        Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);   
        waypointList.add(0, wpShip);         	    	    	    	
    	distanceMatrix = createDistanceMatrix(waypointList);    	    	   	
		long timeAfterMatrix = System.currentTimeMillis();
		System.out.println(" Time spent to build distance matrix: " + (timeAfterMatrix - timeStart) + " ms.");		
 
    	waypointList.remove(wpShip);//remove ship position from search space
    	orderedWaypoints.add(wpShip);//always start from ship    	
       	runList(waypointList, orderedWaypoints);

		long timeAfterSearch = System.currentTimeMillis();
		System.out.println(" Time spent searching: " + (timeAfterSearch - timeAfterMatrix) + " ms.");	
		System.out.println("Path distance:" + pathCostMin);
		System.out.println("Brute Force Search Planner time: " + (timeAfterSearch - timeStart) + " ms.");
    }


	
    /**
     * creates all possible combinations of paths from a given list of waypoints
     * @param aList
     * @param aPath
     * @param depth
     * @param currentCost
     * @return
     */       
    @SuppressWarnings("unchecked")
	private LinkedList<GameObject> runList(LinkedList<GameObject> aList, LinkedList<GameObject> aPath, int depth, double currentCost)
    {
    	if (verbose) System.out.print("\n\nin:" + depth);
    	if (verbose) System.out.print("\naList:");
    	if (verbose) for (GameObject way : aList) {
    		System.out.print(way.getName()  + " ");
    	}
    	if (verbose) System.out.print("\naPath:");
    	if (verbose) for (GameObject way : aPath) {
    		System.out.print(way.getName()  + " ");
    	}
    	LinkedList<GameObject> localPath = new LinkedList<>();
    	LinkedList<GameObject> localList = new LinkedList<>();
    	if (aList.size() > 1) 
    	{
    		for (int i = 0; i < aList.size(); i++)
    		{
    			 localList = (LinkedList<GameObject>) aList.clone();
    			 localList.remove(i);
    			 localPath = (LinkedList<GameObject>) aPath.clone();
    			 localPath.add(aList.get(i));    		
    			 runList(localList, localPath, depth+1, currentCost);//send a deeper depth down, but do not increase local one
    		}
    	} else
    	{
    		aPath.add(aList.get(0));  	
    		double pathCost = getPathDistance(aPath);
    		if (pathCost < pathCostMin)
    		{
    			pathCostMin = pathCost;
    			orderedWaypoints = aPath;
    		}
    	}
    	return aPath;  	
    }
    /**
     * creates all possible combinations of paths from a given list of waypoints
     * @param waypointList
     * @param orderedWaypoints
     * @return
     */   
    private LinkedList<GameObject> runList(LinkedList<GameObject> waypointList, LinkedList<GameObject> orderedWaypoints)
    {
    	return runList(waypointList, orderedWaypoints, 0, 0);
    }

}
