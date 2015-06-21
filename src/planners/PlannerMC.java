package planners;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import framework.core.Game;
import framework.core.GameObject;
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
	long timeDue;
	
	public PlannerMC(Game aGameCopy)
    {
    	this(aGameCopy, 0);
    }
	
    public PlannerMC(Game aGameCopy, int timeAllowed)
    {
    	System.out.println("***monte carlo planner***");
    	this.aGameCopy = aGameCopy;
    	this.timeDue = System.currentTimeMillis() + timeAllowed;
    }
    	
	public void runPlanner()
	{
        long timeStart = System.currentTimeMillis();
    	aGraph = new Graph(aGameCopy);
   	    	
    	LinkedList<GameObject> waypointList;    
    	GameObject wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);
        
        //if this is the main planner, build distance and cost matrices
        if(distanceMatrix.size() == 0)
        {
        	createMatrices();
        }
		long timeAfterMatrices = System.currentTimeMillis();
		System.out.println(" Total time spent for matrices init: " + (timeAfterMatrices - timeStart) + " ms.");		
    	
    	// find the shortest path among the randomly generated paths    	
    	Random rand = new Random();
    	LinkedList<GameObject> aPath = new LinkedList<>();//the random path    	
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
    		waypointList = (LinkedList<GameObject>) aGameCopy.getWaypoints().clone();//start with all the waypoints
    		waypointList.addAll((Collection<? extends GameObject>) aGameCopy.getFuelTanks().clone());//add all the fuel tanks
    		    		
    		//extract a random waypoint and add it to the path
        	while(waypointList.size() > 0)
        	{   
        		//TODO ~ control actual randomness here
//        		int randomIndex = 0;//sequential list
        		rand.setSeed(100);//fixed random list
        		int randomIndex = rand.nextInt(waypointList.size());
        		GameObject thePoppedWay = waypointList.get(randomIndex);
        		waypointList.remove(randomIndex);
        		aPath.add(thePoppedWay);
        	}    	
        	
        	pathCost = getPathCost(aPath);
        	if (verbose) System.out.println(tries + " : generated " + pathCost + "(" + pathMinCost + ")");
        	if(pathCost < pathMinCost)
        	{
        		pathMinCost = pathCost;
        		orderedWaypoints = (LinkedList<GameObject>) aPath.clone();
        	}    	
        	if (System.currentTimeMillis() > timeDue - bufferTime)
        	{
        		theresstilltime = false;
        	}
    	}    	
    	long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent randomizing " + tries + " paths: " + (timeAfter - timeAfterMatrices) + " ms.");    	
		System.out.println("Path distance:" + getPathCost(orderedWaypoints));			
		System.out.println("MC Planner time: " + (timeAfter - timeStart) + " ms.");	
	}
}
