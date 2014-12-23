package controllers.mctsnavi;

import java.util.ArrayList;

public class SearchTreeNode {
	private int action = -1;//action performed that got us here
	private int visited = 0;
	private double score = Double.NEGATIVE_INFINITY;
//	private double value = Double.NEGATIVE_INFINITY;
	private double value = Double.POSITIVE_INFINITY;
	private SearchTreeNode parent = null;
	private ArrayList<SearchTreeNode> children = new ArrayList<>();
	private double alpha = Double.NEGATIVE_INFINITY;
	private double beta = Double.POSITIVE_INFINITY;
	
	public ArrayList<SearchTreeNode> getChildren() {
		return children;
	}
	public void setChildren(ArrayList<SearchTreeNode> children) {
		this.children = children;
	}
	public void addChild(SearchTreeNode a_child)
	{
		children.add(a_child);
	}
	public SearchTreeNode getParent() {
		return parent;
	}
	public void setParent(SearchTreeNode parent) {
		this.parent = parent;
	}
	public int getAction() {
		return action;
	}
	public void setAction(int action) {
		this.action = action;
	}
	public int getVisited() {
		return visited;
	}
	public void setVisited(int visited) {
		this.visited = visited;
	}
	public void incrementVisited()
	{		
		this.visited++;
	}
	public double getScore() {
		return score;
	}
	public void setScore(double score) {
		this.score = score;
		updateValue();
	}
	public void addScore(double score)
	{
		this.score += score;
		updateValue();
	}
	public double getValue() {
		return value;
	}
	private void setValue(double value) {
		this.value = value;
	}	
	private void updateValue()
	{
		this.incrementVisited();
		setValue((double)this.score / (double)this.visited);
	}
	public boolean hasChildObject(SearchTreeNode possibleChild)
	{
		if(children.contains(possibleChild)) return true;
		return false;
	}
	
	public boolean hasChild(SearchTreeNode possibleChild)
	{
		for(SearchTreeNode a_node : children)
		{
			if(a_node.action == possibleChild.action)
				return true;
		}
		return false;
	}
	public SearchTreeNode getChild(int action) {
		for(SearchTreeNode a_node : children)
		{
			if(a_node.action == action)
				return a_node;
		}
		return null;
	}
	
	public void present(int depth)
	{
		System.out.print("\n" +this.hashCode() + "(p:" + (this.parent != null ? this.parent.hashCode() : "-") + ")" + " [V:" + this.value + "] [a:" + this.action + "] [s:" + this.score + "] [v:" + this.visited + "] [c:" + this.getChildren().size()+"]");
		if(this.getChildren().size() > 0)
		{
			depth++;
			for(SearchTreeNode aNode : this.getChildren())
			{
				System.out.print("\n");
				for(int i = 0; i < depth; i++) System.out.print("\t");
				aNode.present(depth);
			}
		}
	}
	public void trySetScore(double actionScore) {
		System.out.println(actionScore + " ? " + this.value);
		// TODO Auto-generated method stub
		if(actionScore < this.value) 
			{
			System.out.println(actionScore + " < " + this.value);
			this.value = actionScore;
			}
	}
	

}
