package planners;

import java.util.LinkedList;
import java.util.Random;

import framework.core.Game;
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
	
    public PlannerBruteForce(Game a_gameCopy)
    {
    	System.out.println("***brute force search planner***");
        long timeStart = System.currentTimeMillis();    	
    	m_graph = new Graph(a_gameCopy);

    	//add ship position as waypoint for distance matrix
    	@SuppressWarnings("unchecked")
		LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();    	    	
        Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);   
        waypointList.add(0, wpShip);         	    	    	    	
    	distanceMatrix = createDistanceMatrix(waypointList);    	    	   	
		long timeAfterMatrix = System.currentTimeMillis();
		System.out.println(" Time spent to build distance matrix: " + (timeAfterMatrix - timeStart) + " ms.");		
 
    	waypointList.remove(wpShip);//remove ship position from search space
    	m_orderedWaypoints.add(wpShip);//always start from ship    	
       	runList(waypointList, m_orderedWaypoints);

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
	private LinkedList<Waypoint> runList(LinkedList<Waypoint> aList, LinkedList<Waypoint> aPath, int depth, double currentCost)
    {
    	if (verbose) System.out.print("\n\nin:" + depth);
    	if (verbose) System.out.print("\naList:");
    	if (verbose) for (Waypoint way : aList) {
    		System.out.print(way.getName()  + " ");
    	}
    	if (verbose) System.out.print("\naPath:");
    	if (verbose) for (Waypoint way : aPath) {
    		System.out.print(way.getName()  + " ");
    	}
    	LinkedList<Waypoint> localPath = new LinkedList<>();
    	LinkedList<Waypoint> localList = new LinkedList<>();
    	if (aList.size() > 1) 
    	{
    		for (int i = 0; i < aList.size(); i++)
    		{
    			 localList = (LinkedList<Waypoint>) aList.clone();
    			 localList.remove(i);
    			 localPath = (LinkedList<Waypoint>) aPath.clone();
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
    			m_orderedWaypoints = aPath;
    		}
    	}
    	return aPath;  	
    }
    /**
     * creates all possible combinations of paths from a given list of waypoints
     * @param aList
     * @param aPath
     * @return
     */   
    private LinkedList<Waypoint> runList(LinkedList<Waypoint> aList, LinkedList<Waypoint> aPath)
    {
    	return runList(aList, aPath, 0, 0);
    }

}
