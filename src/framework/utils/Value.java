package framework.utils;

public class Value {
	public double distance;
	public double direction;
	
	public Value()
	{
		reset();
	}
	
	public void reset() {
		this.distance = Double.MAX_VALUE;
		this.direction= Double.MIN_VALUE;
	}
	
}