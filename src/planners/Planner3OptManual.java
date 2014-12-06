package planners;

import java.util.LinkedList;
import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  TODO:
 *  - this is the worst approach
 *  - if it works ,fuck it, use it
 *  @author Cristian
 *  @version 141204
 */

public class Planner3OptManual extends Planner {

    @SuppressWarnings("unchecked")
	public Planner3OptManual(Game a_gameCopy)
    {
    	System.out.println("***manual planner***");
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
//        waypointList.addFirst(wpShip);
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
		int rem1_s = 0, rem1_e = 0, rem2_s = 0, rem2_e = 0 , rem3_s = 0, rem3_e = 0;		
        
		//full search through all edges
		for (int i = 1; i < tempLimit; i++)//first break
		{
			for (int j = i+2; j < tempLimit; j++)//second break +2 to avoid removing adjacent edges, as that creates a closed loop
    		{
				for (int k = j+2; k < tempLimit; k++)//third break +2 to avoid removing adjacent edges, as that creates a closed loop
	    		{
	                intPath.clear();
	                intPathRev.clear();
					//	edges to remove    				
	    			if (verbose) 
	    			{
	    				System.out.print("===removed " + (i-1) + "~" + (i));
	    				rem1_s = i-1;
	    				rem1_e = i;
	    				System.out.print(" removed " + (j-1) + "~" + (j));
	    				rem2_s = j-1;
	    				rem2_e = j;
	    				System.out.println(" removed " + (k-1) + "~" + (k));
	    				rem3_s = k-1;
	    				rem3_e = k;
	    			}
	    		}
    		}
    	}
			
		//rebuild
        intPath.clear();		
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_s, rem1_e, false, intPath);	
				walkPath(rem2_e, rem3_s, true, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);
		
        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_s, rem1_e, false, intPath);	
				walkPath(rem3_s, rem2_e, false, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				

		
        intPath.clear();			
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_e, rem3_s, true, intPath);	
				walkPath(rem1_e, rem2_s, true, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				

        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_e, rem3_s, true, intPath);	
				walkPath(rem2_s, rem1_e, false, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				
	

        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem3_s, rem2_e, false, intPath);	
				walkPath(rem1_e, rem2_s, true, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				
		
        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem3_s, rem2_e, false, intPath);	
				walkPath(rem2_s, rem1_e, false, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				
			
			
		System.out.println("\nreversed direction");
		for (int i = tempLimit; i > 1; i--)
		{
			for (int j = i-2; j > 1; j--)//+2 to avoid removing adjacent edges, as that creates a closed loop
    		{
				for (int k = j-2; k > 1; k--)//+2 to avoid removing adjacent edges, as that creates a closed loop
	    		{
	                intPath.clear();
	                intPathRev.clear();
					//	remove edges    				
	    			if (verbose) 
	    			{
	    				System.out.print("removed " + (0) + "~" + (i-1));
	    				rem1_s = 0;
	    				rem1_e = i-1;
	    				System.out.print(" removed " + (j-1) + "~" + (j));
	    				rem2_s = j-1;
	    				rem2_e = j;
	    				System.out.println(" removed " + (k-1) + "~" + (k));
	    				rem3_s = k-1;
	    				rem3_e = k;
	    			}	    			
	    		}
    		}
    	} 		
		
		
		//rebuild
        intPath.clear();		
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_s, rem1_e, false, intPath);	
				walkPath(rem2_e, rem3_s, true, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);
		
        intPath.clear();
		walkPath(tempLimit-1, rem1_s, false, intPath);
			walkPath(rem2_s, rem1_e, false, intPath);	
				walkPath(rem3_s, rem2_e, false, intPath);
		walkPath(rem3_e, 1, false, intPath);
		System.out.println(intPath);				

		
        intPath.clear();			
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_e, rem3_s, true, intPath);	
				walkPath(rem1_e, rem2_s, true, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				

        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem2_e, rem3_s, true, intPath);	
				walkPath(rem2_s, rem1_e, false, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				
	

        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem3_s, rem2_e, false, intPath);	
				walkPath(rem1_e, rem2_s, true, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);				
		
        intPath.clear();
		walkPath(1, rem1_s, true, intPath);
			walkPath(rem3_s, rem2_e, false, intPath);	
				walkPath(rem2_s, rem1_e, false, intPath);
		walkPath(rem3_e, tempLimit-1, true, intPath);
		System.out.println(intPath);	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
    	long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent searching: " + (timeAfter - timeAfterMatrix) + " ms.");    	
		System.out.println("Path distance:" + getPathDistance(m_orderedWaypoints));			
		System.out.println("3Opt Planner time: " + (timeAfter - timeStart) + " ms.");	
		System.exit(0);
    }

    private void walkPath(int from, int to, boolean forward, LinkedList<Integer> currentPath)
    {
    	if(forward) 
    	{
    		for (int i = from; i <= to; i ++)
	    	{
	    		currentPath.add(i);
	    	}    		
    	}
	    
    	else 
    	{
    		for (int i = from; i >= to; i --)
	    	{
	    		currentPath.add(i);
	    	}
    	}
    		
    }
    
}
