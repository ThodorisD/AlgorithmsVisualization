package BellmanFord;

import java.util.List;

public class BellmanFordSP{

	private List<Edge> edgeList ;
	private List<Vertex> vertexList ;
	
	
	public BellmanFordSP(List<Edge> edgeList,List<Vertex> vertexList) {
		this.edgeList = edgeList ;
		this.vertexList = vertexList ;
	}
	
	public void startBellmanFord(Vertex sourceVertex) {
		
		sourceVertex.setDistance(0);
		
		for(int i= 0 ; i < vertexList.size() -1 ; i++) {
			
			for(Edge edge : edgeList) {
				
				Vertex u = edge.getStartVertex() ;
				Vertex v = edge.getTargetVertex() ;
				
				if(u.getDistance() == Double.MAX_VALUE)
					continue ;
				
				double newDistance = u.getDistance() + edge.getWeight() ;
				
				if(newDistance < v.getDistance()) {
					v.setDistance(newDistance);
					v.setPreviousVertex(u);
				}
			}
		}
		
		for(Edge edge :edgeList) {
			if(edge.getStartVertex().getDistance() != Double.MAX_VALUE) {
				if(hasCycle(edge)) {
					System.out.println("There has been a negative cycle detected...");
					return ;
				}
			}
		}
	}

	private boolean hasCycle(Edge edge) {
		return edge.getStartVertex().getDistance() + edge.getWeight() < edge.getTargetVertex().getDistance() ;
	}
	
	public void shortestPathTo(Vertex targetVertex){
		if(targetVertex.getDistance() == Double.MAX_VALUE) {
			System.out.println("There is no path from the source to the target.");
		}
		
		Vertex actualVertex = targetVertex ;
		
		while(actualVertex.getPreviousVertex() != null) {
			System.out.print(actualVertex + " - ");
			actualVertex = actualVertex.getPreviousVertex() ;
		}
		
	}
}
