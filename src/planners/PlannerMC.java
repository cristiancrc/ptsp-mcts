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
	
	int bufferTime = 5;
	
    @SuppressWarnings("unchecked")
    public PlannerMC(Game a_gameCopy, int timeAllowed)
    {
    	this(a_gameCopy, System.currentTimeMillis() + timeAllowed);    
    }
	public PlannerMC(Game a_gameCopy, long timeDue)
    {
    	System.out.println("***monte carlo planner***");
        long timeStart = System.currentTimeMillis();
        this.aGameCopy = a_gameCopy;
    	m_graph = new Graph(a_gameCopy);
   	    	
    	LinkedList<Waypoint> waypointList;    
        Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);
        
        //if this is the main planner, build distance and cost matrices
        if(distanceMatrix.size() == 0)
        {
        	createMatrices();
        }
		long timeAfterMatrices = System.currentTimeMillis();
		System.out.println(" Total time spent for matrices init: " + (timeAfterMatrices - timeStart) + " ms.");		
    	
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
    		aPath.add(wpShip);//add ship position as waypoint, to be included in the distance matrix
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
        	if (System.currentTimeMillis() > timeDue - bufferTime)
        	{
        		theresstilltime = false;
        	}
    	}    	
    	long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent randomizing " + tries + " paths: " + (timeAfter - timeAfterMatrices) + " ms.");    	
		System.out.println("Path distance:" + getPathCost(m_orderedWaypoints));			
		System.out.println("MC Planner time: " + (timeAfter - timeStart) + " ms.");	
    }
}
