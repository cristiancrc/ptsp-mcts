package planners;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  Monte Carlo planner
 *  adds waypoints randomly and keeps the shortest list
 *  @author Cristian
 *  @version 141202
 */

public class PlannerMC extends Planner {
	
    @SuppressWarnings("unchecked")
    public PlannerMC(Game a_gameCopy)
    {
    	this(a_gameCopy, 800);
    }
	public PlannerMC(Game a_gameCopy, int allotedTime)
    {
    	System.out.println("***monte carlo planner***");
        long timeStart = System.currentTimeMillis();
    	m_graph = new Graph(a_gameCopy);

    	LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();    
        Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);        
        waypointList.add(0, wpShip);//add ship position as waypoint, to be included in the distance matrix
    	HashMap<Waypoint, HashMap<Waypoint, Double>>[] distanceMatrices = createDistanceMatrices(waypointList);
    	distanceMatrix = distanceMatrices[0];
    	matrixCostLava = distanceMatrices[1];  	
		long timeAfterMatrix = System.currentTimeMillis();
		System.out.println(" Time spent to build distance matrix: " + (timeAfterMatrix - timeStart) + " ms.");		
    	
    	// find the shortest path among the randomly generated paths
    	
    	Random rand = new Random();
    	LinkedList<Waypoint> aPath = new LinkedList<>();//the random path    	
    	double pathMinCost = Double.MAX_VALUE;    	
    	int tries = 0;    	    	
    	boolean theresstilltime = true;    	
    	while(theresstilltime)
    	{
    		tries++;
    		//new path, starting from the ship
    		double pathCost = Double.MAX_VALUE;    		
    		aPath.clear();
    		aPath.add(wpShip);
    		waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();//start with all the waypoints
    		
    		//extract a random waypoint and add it to the path
        	while(waypointList.size() > 0)
        	{        		
        		int randomIndex = rand.nextInt(waypointList.size());        		
        		Waypoint thePoppedWay = waypointList.get(randomIndex);
        		waypointList.remove(thePoppedWay);
        		aPath.add(thePoppedWay);
        	}    	
        	
        	pathCost = getPathCost(aPath);
        	if (verbose) System.out.println(tries + " : generated " + pathCost + "(" + pathMinCost + ")");
        	if(pathCost < pathMinCost)
        	{
        		pathMinCost = pathCost;
        		m_orderedWaypoints = (LinkedList<Waypoint>) aPath.clone();
        	}    		    
        	if (System.currentTimeMillis() - timeStart > allotedTime)
        	{
        		theresstilltime = false;
        	}
    	}    	
    	long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent randomizing " + tries + " paths: " + (timeAfter - timeAfterMatrix) + " ms.");    	
		System.out.println("Path distance:" + getPathCost(m_orderedWaypoints));			
		System.out.println("MC Planner time: " + (timeAfter - timeStart) + " ms.");	
    }
}
