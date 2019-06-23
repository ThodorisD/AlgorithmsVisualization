package basicgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import util.GraphLoader;

public abstract class Graph {

	private int numVertices;
	private int numEdges;
	private Map<Integer,String> vertexLabels;

	public Graph() {
		numVertices = 0;
		numEdges = 0;
		vertexLabels = null;
	}


	public int getNumVertices() {
		return numVertices;
	}
	

	public int getNumEdges() {
		return numEdges;
	}

	public int addVertex() {
		implementAddVertex();
		numVertices ++;
		return (numVertices-1);
	}

	public abstract void implementAddVertex();
	

	public void addEdge(int v , int w) {
		numEdges ++;
		if (v < numVertices && w < numVertices) {
			implementAddEdge(v , w);			
		}
		else {
			throw new IndexOutOfBoundsException();
		}
	}

	public abstract void implementAddEdge(int v, int w);
	

	public abstract List<Integer> getNeighbors(int v); 

	public abstract List<Integer> getInNeighbors(int v);
	

	public List<Integer> degreeSequence() {
		List<Integer> degreeSequence = new ArrayList<>();
		for (int i = 0; i < getNumVertices(); i++) {
			Integer outNeighbors = getNeighbors(i).size();
			Integer inNeighbors = getInNeighbors(i).size();
			degreeSequence.add(outNeighbors + inNeighbors);
		}
		Collections.sort(degreeSequence, Collections.reverseOrder());

		return degreeSequence;
	}

	public abstract List<Integer> getDistance2(int v); 

	public String toString() {
		String s = "\nGraph with " + numVertices + " vertices and " + numEdges + " edges.\n";
		s += "Degree sequence: " + degreeSequence() + ".\n";
		if (numVertices <= 20) s += adjacencyString();
		return s;
	}

	public abstract String adjacencyString();

	
	// The next methods implement labeled vertices.
	// Basic graphs may or may not have labeled vertices.

	public void initializeLabels() {
		vertexLabels = new HashMap<Integer,String>();
	}	

	public boolean hasVertex(int v)
	{
		return v < getNumVertices();
	}

	public boolean hasVertex(String s)
	{
		return vertexLabels.containsValue(s);
	}
	

	public void addLabel(int v, String s) {
		if (v < getNumVertices() && !vertexLabels.containsKey(v)) 
		{
			vertexLabels.put(v, s);
		}
		else {
			System.out.println("ERROR: tried to label a vertex that is out of range or already labeled");
		}
	}
	

	public String getLabel(int v) {
		if (vertexLabels.containsKey(v)) {
			return vertexLabels.get(v);
		}
		else return null;
	}

	public int getIndex(String s) {
		for (Map.Entry<Integer,String> entry : vertexLabels.entrySet()) {
			if (entry.getValue().equals(s))
				return entry.getKey();
		}
		System.out.println("ERROR: No vertex with this label");
		return -1;
	}

}
