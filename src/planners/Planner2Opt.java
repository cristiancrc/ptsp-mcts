package planners;

import java.util.HashMap;
import java.util.LinkedList;

import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  2 opt planner
 *  searches by going both ways through the nodes and removing two edges
 *  @author Cristian
 *  @version 141204
 */

public class Planner2Opt extends Planner {

    @SuppressWarnings("unchecked")
	public Planner2Opt(Game aGameCopy)
    {
    	System.out.println("***2 opt planner***");
    	this.aGameCopy = aGameCopy;
    }
    
    public void runPlanner()
    {
        long timeStart = System.currentTimeMillis();
    	aGraph = new Graph(aGameCopy);    	
    	createMatrices();
        long timeAfterMatrices = System.currentTimeMillis();
		System.out.println(" Total time spent for matrices init: " + (timeAfterMatrices - timeStart) + " ms.");
    	//get a greedy plan
		Planner planner = new PlannerGreedy(aGameCopy);
    	LinkedList<GameObject> waypointList = (LinkedList<GameObject>) aGameCopy.getWaypoints().clone();//the list of waypoints
		waypointList = planner.getOrderedWaypoints();//get the planned route
		long timeAfterGreedy = System.currentTimeMillis();
		System.out.println(" Time spent for greedy planner: " + (timeAfterGreedy - timeAfterMatrices) + " ms.");		
    	Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);//add ship position as waypoint        

		//build paths, based on the greedy result
    	double pathMinCost = getPathCost(waypointList);//result from greedy	
		double pathCost = 0;//local search result
		LinkedList<GameObject> aPath = new LinkedList<>();//stores built paths
		LinkedList<GameObject> aPathRev = new LinkedList<>();//stores built paths

		//full search through all edges
		for (int i = 1; i < waypointList.size()-2; i++)//no need to check the last, 1 because counting starts at 0, another 1 because this is a graph not a circuit
		{
			for (int j = i+2; j < waypointList.size(); j++)//+2 to avoid removing adjacent edges, as that creates a closed loop
    		{
    			aPath.clear();
    			aPathRev.clear();

				//	remove edges    				
    			if (verbose) 
    			{
    				System.out.println("removed " + (i-1) + " ~ " + (i));
    				System.out.println("removed " + (j-1) + " ~ " + (j));
    			}
				
    			// add the nodes of the first part, up to the first break
    			if (verbose) System.out.print("\tadding nodes before break");	
				for (int m = 0; m < i; m++)
    			{
					if (verbose) System.out.print(" " + m);
    				aPath.add(waypointList.get(m));
    			}
				
				if (verbose) System.out.println("\n\t reconnecting");
				//  add the nodes of the first part
				for(int l = j-1; l > i; l--)
				{
					if (verbose) System.out.println("\t added " + l);
					aPath.add(waypointList.get(l));
				}
				// reconnect with second part
				if (verbose) System.out.println("\t added " + (i) + " ~ " + (j));
				aPath.add(waypointList.get(i));
				aPath.add(waypointList.get(j));
				
				// add the nodes of the second part
				if (verbose) System.out.print("\tadding rest ");
				for(int k = j+1; k < waypointList.size(); k++)
				{
					if (verbose) System.out.print(" " + (k));	
					aPath.add(waypointList.get(k));
				}
				//create reversed list
				for (int c = aPath.size()-1; c > 0; c--)
				{
					aPathRev.add(aPath.get(c));
				}

				// check if one of the resulting paths is shorter than minimum
//				aPath.addFirst(wpShip);
				if (verbose) presentList(aPath);
            	pathCost = getPathCost(aPath);
            	if (verbose) System.out.println("\n generated " + pathCost + "(" + pathMinCost + ")");
            	if(pathCost < pathMinCost)
            	{
            		pathMinCost = pathCost;
            		orderedWaypoints = (LinkedList<GameObject>) aPath.clone();
            	}  					
            	
            	
				aPathRev.addFirst(wpShip);
				if (verbose) presentList(aPathRev);
            	pathCost = getPathCost(aPathRev);
            	if (verbose) System.out.println(" generated " + pathCost + "(" + pathMinCost + ")");
            	if(pathCost < pathMinCost)
            	{
            		pathMinCost = pathCost;
            		orderedWaypoints = (LinkedList<GameObject>) aPathRev.clone();
            	}  							
    		}
			if (verbose) System.out.println("");   		
    	}    	
    	long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent searching: " + (timeAfter - timeAfterGreedy) + " ms.");    	
		System.out.println("Path distance:" + getPathCost(orderedWaypoints));			
		System.out.println("2Opt Planner time: " + (timeAfter - timeStart) + " ms.");	
    }
}
