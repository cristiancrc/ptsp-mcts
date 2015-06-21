package planners;

import java.util.HashMap;
import java.util.LinkedList;

import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  TODO: implement after all else is done
 *  - do not delete, some code is tried in here
 *  - based on working 3Opt
 *  - find a general solution, most probably recursive calls
 *  - remove first path, as its always the one without any removals (solved in 3opt)
 *  - consider generating up to half of the list and then reverse the result
 *  @author Cristian
 *  @version 141208
 */

public class PlannerKOpt extends Planner {
	
	private int Kopt;

	@SuppressWarnings("unchecked")
	public PlannerKOpt(Game aGameCopy, int Kopt)
    {
    	System.out.println("***KOpt planner***");
    	System.out.println("running with K = " + Kopt);
    	this.aGameCopy = aGameCopy;
    	this.Kopt = Kopt;
    }
	
	public void runPlanner()
	{
        long timeStart = System.currentTimeMillis();
    	aGraph = new Graph(aGameCopy);
    	
    	verbose = true;
    	
    	//get a greedy plan
		Planner planner = new PlannerGreedy(aGameCopy);
		planner.runPlanner();
    	LinkedList<GameObject> waypointList = (LinkedList<GameObject>) aGameCopy.getWaypoints().clone();//the list of waypoints
		waypointList = planner.getOrderedWaypoints();//get the planned route
		
		long timeAfterGreedy = System.currentTimeMillis();
		System.out.println(" Time spent for greedy planner: " + (timeAfterGreedy - timeStart) + " ms.");		

		//add ship position as waypoint
    	Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);        
    	orderedWaypoints.add(wpShip);
    	HashMap<GameObject, HashMap<GameObject, Double>>[] distanceMatrices = createDistanceMatrices(waypointList);
    	distanceMatrix = distanceMatrices[0];
    	matrixCostLava = distanceMatrices[1];            	
		long timeAfterMatrix = System.currentTimeMillis();
		System.out.println(" Time spent to build distance matrix: " + (timeAfterMatrix - timeAfterGreedy) + " ms.");
		
		//build paths, based on the greedy result
    	double pathMinCost = getPathCost(waypointList);//result from greedy	
		double pathCost = 0;//local search result
		LinkedList<GameObject> aPath = new LinkedList<>();//stores built paths
		
		//reduce number of waypoints
		int tempLimit = waypointList.size();
		tempLimit = 11;	// unlimited is 11	
		if(tempLimit != waypointList.size())
		{
			System.out.println("limiting waypoint list to " + tempLimit);
			//actually reduce no of waypoints
			LinkedList<GameObject> shortList = new LinkedList<>();
			for (int i = 0; i < tempLimit; i++)
			{
				shortList.add(waypointList.get(i));
			}
			waypointList = shortList;			
		}
		
		// find the possible edges than can be removed
		/*
		 * for the latest Kopt tour, the earliest break can start no later than
		 * the total number of nodes 
		 * minus the number of removed edges (Kopt)
		 * minus the number of non-removed edges between the removed edges (Kopt-1)
		 */
		int latestBreak1Start = waypointList.size() - Kopt - (Kopt - 1);
		// if kopt is set too high for the number of waypoints, overwrite and use the maximum kopt possible
		if(latestBreak1Start < 0) 
		{
			System.out.println("Kopt parameter too high (" + Kopt + "), for " + waypointList.size() + " waypoints, only " + ((waypointList.size()-1)/2) + " edges can be removed ");
//			Kopt = ((waypointList.size()-1)/2);
//			latestBreakStart = waypointList.size() - Kopt - (Kopt - 1);
		}
		
		//3OPT break points correct
		int b1_s = 0, b1_e = 0, b2_s = 0, b2_e = 0 , b3_s = 0, b3_e = 0;			
		for (int i = 0; i < tempLimit; i++) //i denotes first break
		{
			for (int j = i+2; j < tempLimit; j++) //j denotes second break
			{
				for (int k = j+2; k < tempLimit; k++) //k denotes third break
				{
//					System.out.println("===i " + i + ", j " + j + ", k " + k);
					//last item in cycle is also the first
					int showK = k+1;
					if(showK == tempLimit) showK = 0;
					if (verbose) System.out.print("...removed " + i + "~" + (i+1) + " " + waypointList.get(i).getName() + "~" + waypointList.get(i+1).getName());
					b1_s = i;
					b1_e = i+1;
					
					if (verbose) System.out.print("... removed " + j + "~" + (j+1) + " " + waypointList.get(j).getName() + "~" + waypointList.get(j+1).getName());
					b2_s = j;
					b2_e = j+1;
					
					if (verbose) System.out.println("... removed " + k + "~" + (k+1) + " " + waypointList.get(k).getName() + "~" + waypointList.get(showK).getName());
					b3_s = k;
					b3_e = k+1;

					//remaining edges, both ways
					HashMap<Integer, Integer> edges = new HashMap<>();
					edges.put(b1_e, b2_s);
					edges.put(b2_s, b1_e);
					
					edges.put(b2_e, b3_s);
					edges.put(b3_s, b2_e);
					
					// 3OPT rebuild cycle
			        LinkedList<Integer> intPath = new LinkedList<>();

					for (int start1 : edges.keySet())
					{
						int end1 = edges.get(start1);
//						System.out.println("edge1 " + start1 + " to " + end1);
						boolean edge1Fwd = (start1 < end1 ? true : false);
						for (int start2 : edges.keySet())
						{
							int end2 = edges.get(start2);
							if(	start2 == start1 //edge taken by previous for loop
							||	start2 == end1 //avoid the reversed edge of the previous for loop
							) continue;
//							System.out.println(" edge2 " + start2 + " to " + end2);
							boolean edge2Fwd = (start2 < end2 ? true : false);
							
							//build int path
					        intPath.clear();
							walkIntPath(1, b1_s, true, intPath);
							walkIntPath(start1, end1, edge1Fwd, intPath);
							walkIntPath(start2, end2, edge2Fwd, intPath);
							walkIntPath(b3_e, tempLimit-1, true, intPath);
//							System.out.println(intPath);
							
							
							//rebuild path based on indexes
							aPath.clear();
							for (int w : intPath)
							{
								aPath.add(waypointList.get(w));
							}
							aPath.addFirst(wpShip);
							if (verbose) presentList(aPath);						
							pathCost = getPathCost(aPath);
			            	if (verbose) System.out.println(" generated " + pathCost + "(" + pathMinCost + ")");
			            	if(pathCost < pathMinCost)
			            	{
			            		pathMinCost = pathCost;
			            		orderedWaypoints = (LinkedList<GameObject>) aPath.clone();
			            	}  								
						}
					}
				}
			}
		}	
		
				
	//3OPT break points correct 
//		//base of a recursive function that finds the break points
//		for (int i = 0; i <= latestBreak1Start; i++)
//		{
////			System.out.println("i(0:" + (latestBreak1Start) + ") , now at: " + i);
//			int b1s = i;
//			int b1e = b1s + 1;
//			System.out.println("break " + " from " + (b1s) + "~" + (b1e));
//			
//			int latestBreak2Start = waypointList.size() - (Kopt-1) - (Kopt-2);
//			for (int j = b1e + 1; j <= latestBreak2Start; j++)
//			{
////				System.out.println(" j(" + (b1e + 1) + ":" + (latestBreak2Start) + ") , now at: " + j);
//				int b2s = j;
//				int b2e = b2s + 1;
//				System.out.println(" break " + " from " + (b2s) + "~" + (b2e));
//				
//				int latestBreak3Start = waypointList.size() - (Kopt-2) - (Kopt-3);
//				for (int k = b2e + 1; k <= latestBreak3Start; k++)
//				{
////					System.out.println(" k(" + (b2e + 1) + ":" + (latestBreak3Start) + ") , now at: " + k);
//					int b3s = k;
//					int b3e = b3s + 1;
//					System.out.println("  break " + " from " + (b3s) + "~" + (b3e));			
//				}
//			}				
//		}			
		
		
		
		
		long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent searching: " + (timeAfter - timeAfterMatrix) + " ms.");    	
		System.out.println("Path distance:" + getPathCost(orderedWaypoints));			
		System.out.println("KOpt Planner time: " + (timeAfter - timeStart) + " ms.");	
		System.exit(0);
    }
	
	/**
	 * default constructor, uses Kopt = 3
	 * @param a_gameCopy
	 */
	public PlannerKOpt(Game a_gameCopy)
    {
		this(a_gameCopy, 3); 
    }	
	
	/**
	 * adds values to currentPath, starting From, up To, going Forward
	 * @param from
	 * @param to
	 * @param forward
	 * @param currentPath
	 */
    private void walkIntPath(int from, int to, boolean forward, LinkedList<Integer> currentPath)
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
