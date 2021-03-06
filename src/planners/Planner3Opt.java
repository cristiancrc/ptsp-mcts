package planners;

import java.util.HashMap;
import java.util.LinkedList;

import controllers.mcts.DriveMCTS;

import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;

/**
 *  TODO 9 consider generating up to half of the list and then reverse the result
 *  @author Cristian
 *  @version 150521
 */

public class Planner3Opt extends Planner {

	public Planner3Opt(Game aGameCopy)
	{
		System.out.println("***3Opt planner***");
		this.aGameCopy = aGameCopy;
	}
	
	public void runPlanner()
    {    	
        long timeStart = System.currentTimeMillis();      
    	aGraph = new Graph(aGameCopy);    	
    	LinkedList<GameObject> waypointList;
    	Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);
    	long timeInit = System.currentTimeMillis();
    	System.out.println(" Time to init: " + (timeInit - timeStart) + " ms.");
    	createMatrices();  	    	
        long timeAfterMatrices = System.currentTimeMillis();
		System.out.println(" Total time spent for matrices init: " + (timeAfterMatrices - timeInit) + " ms.");
		
		int allotedTime = 0;
		if(includeFuel)
		{
			allotedTime = 300;//with fuel
		}
		else
		{
			allotedTime = 500;//without fuel
		}
    	//get a base plan		
//		Planner planner = new PlannerGreedy(a_gameCopy);
    	Planner planner = new PlannerMC(aGameCopy, allotedTime);
    	planner.runPlanner();
		waypointList = planner.getOrderedWaypoints();//get the planned route
//		System.out.print("out from DriveMC_w:");
//		presentList(waypointList);
//		System.out.println("\n");
		
		orderedWaypoints = (LinkedList<GameObject>) waypointList.clone();//an initial list is needed
		long timeAfterBasicPlanner = System.currentTimeMillis();
		
		System.out.println(" Time spent for base planner: " + (timeAfterBasicPlanner - timeAfterMatrices) + " ms.");		
			
		//build paths, based on the basic result
    	double pathMinCost = getPathCost(waypointList);//result from basic planner
    	System.out.println(" Initial path cost: " + pathMinCost);
		double pathCost = 0;//local search result
		LinkedList<GameObject> aPath = new LinkedList<>();//stores built paths
		
		//for debug, it is possible to reduce number of waypoints
		int searchLimit = waypointList.size();
//		searchLimit = 5;	// use a subset	
		if(searchLimit != waypointList.size())
		{
			System.out.println("limiting waypoint list to " + searchLimit);
			//actually reduce no of waypoints
			LinkedList<GameObject> shortList = new LinkedList<>();
			for (int i = 0; i < searchLimit; i++)
			{
				shortList.add(waypointList.get(i));
			}
			waypointList = shortList;			
		}
		long timeBeforeSearching = System.currentTimeMillis();
		//3OPT three break points, start and end position
		int b1_s = 0, b1_e = 0, b2_s = 0, b2_e = 0 , b3_s = 0, b3_e = 0;			
		for (int i = 0; i < searchLimit; i++) //i denotes first break
		{
			for (int j = i+2; j < searchLimit; j++) //j denotes second break
			{
				for (int k = j+2; k < searchLimit; k++) //k denotes third break
				{
//					System.out.println("===i " + i + ", j " + j + ", k " + k);
					//last item in cycle is also the first
					int showK = k+1;
					if(showK == searchLimit) showK = 0;
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
			        LinkedList<Integer> intPath = new LinkedList<>();//a list of the waypoint indexes
			        boolean firstPathSkipped = false;
					for (int start1 : edges.keySet())
					{
						int end1 = edges.get(start1);
//						System.out.println("edge1 " + start1 + " to " + end1);
						boolean edge1Fwd = (start1 < end1 ? true : false);
//						
						for (int start2 : edges.keySet())
						{
							int end2 = edges.get(start2);
							//skip taken edge and its reverse
							if(	start2 == start1 //edge taken by previous for loop
							||	start2 == end1 //avoid the reversed edge of the previous for loop
							) continue;
//							System.out.println(" edge2 " + start2 + " to " + end2);
							boolean edge2Fwd = (start2 < end2 ? true : false);

							//skip the basic rebuild (none of the edges are actually removed)
							if(!firstPathSkipped)
							{								
								firstPathSkipped = true;
								continue;
							}
							//rebuild path with indexes
					        intPath.clear();
							walkIntPath(1, b1_s, true, intPath);
							walkIntPath(start1, end1, edge1Fwd, intPath);
							walkIntPath(start2, end2, edge2Fwd, intPath);
							walkIntPath(b3_e, searchLimit-1, true, intPath);
//							System.out.println(intPath);//all resulting paths

							//rebuild path based on indexes
							aPath.clear();
							for (int w : intPath)
							{
								aPath.add(waypointList.get(w));
							}
							aPath.addFirst(wpShip);
							if (verbose) presentList(aPath);
							//alternate cost function when fuel tanks are in use
							pathCost = getPathCost(aPath);	
			            	if (verbose) System.out.println(" generated " + pathCost + "(" + pathMinCost + ")");
			            	if(pathCost < pathMinCost)
			            	{
			            		pathMinCost = pathCost;
			            		orderedWaypoints = (LinkedList<GameObject>) aPath.clone();
			            	}
			            	// else better have an initial list
						}
					}
				}
			}
		}
		long timeAfter = System.currentTimeMillis();
    	System.out.println(" Time spent 3opt searching: " + (timeAfter - timeBeforeSearching) + " ms.");    	
		System.out.println("Final path cost:" + getPathCost(orderedWaypoints));
		System.out.println("3Opt Planner time: " + (timeAfter - timeStart) + " ms.");
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
