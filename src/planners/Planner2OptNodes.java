package planners;

import java.util.LinkedList;
import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  2 opt planner
 *  searches by going both ways through the nodes and removing two edges
 *  @author Cristian
 *  @version 141204
 */

public class Planner2OptNodes extends Planner {

    @SuppressWarnings("unchecked")
	public Planner2OptNodes(Game a_gameCopy)
    {
    	System.out.println("***2 opt planner***");
        long timeStart = System.currentTimeMillis();
    	m_graph = new Graph(a_gameCopy);
    	verbose = true;
    	
    	//get a greedy plan
		Planner planner = new PlannerGreedy(a_gameCopy);
    	LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();//the list of waypoints
		waypointList = planner.getOrderedWaypoints();//get the planned route
		long timeAfterGreedy = System.currentTimeMillis();
		System.out.println(" Time spent for greedy planner: " + (timeAfterGreedy - timeStart) + " ms.");		

    	Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);//add ship position as waypoint        
        waypointList.addFirst(wpShip);
    	m_orderedWaypoints.add(wpShip);
    	distanceMatrix = createDistanceMatrix(waypointList);            	
		long timeAfterMatrix = System.currentTimeMillis();
		System.out.println(" Time spent to build distance matrix: " + (timeAfterMatrix - timeAfterGreedy) + " ms.");		
		

		//build paths, based on the greedy result
    	double pathMinCost = getPathDistance(waypointList);//result from greedy	
		double pathCost = 0;//local search result
		LinkedList<Waypoint> aPath = new LinkedList<>();//stores built paths
		LinkedList<Waypoint> aPathRev = new LinkedList<>();//stores built paths

				
		
		//full search through all edges
		for (int i = 0; i < waypointList.size()-2; i++)//no need to check the last, 1 because counting starts at 0, another 1 because this is a graph not a circuit
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
				for (int c = aPath.size()-1; c >= 0; c--)
				{
					aPathRev.add(aPath.get(c));
				}

				// check if one of the resulting paths is shorter than minimum
				aPath.addFirst(wpShip);
            	pathCost = getPathDistance(aPath);
            	if (verbose) System.out.println("\n generated " + pathCost + "(" + pathMinCost + ")");
            	if(pathCost < pathMinCost)
            	{
            		pathMinCost = pathCost;
            		m_orderedWaypoints = (LinkedList<Waypoint>) aPath.clone();
            	}  					
            	
				aPathRev.addFirst(wpShip);
            	pathCost = getPathDistance(aPathRev);
            	if (verbose) System.out.println(" generated " + pathCost + "(" + pathMinCost + ")");
            	if(pathCost < pathMinCost)
            	{
            		pathMinCost = pathCost;
            		m_orderedWaypoints = (LinkedList<Waypoint>) aPathRev.clone();
            	}  							
    		}
			if (verbose) System.out.println("");   		
    	}    	
    	long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent searching: " + (timeAfter - timeAfterMatrix) + " ms.");    	
		System.out.println("Path distance:" + getPathDistance(m_orderedWaypoints));			
		System.out.println("2Opt Planner time: " + (timeAfter - timeStart) + " ms.");	
		System.exit(0);
    }
}
