package parser;

import exception.FileFormatException;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import algorithms.DirectedEdge;
import algorithms.EdgeWeightedDigraph;

public class DataParser {

    private EdgeWeightedDigraph  graph ;
     
    // Path to file that contains the graph data (Nodes,Edges,Weights).
	public static String DATASET = ""; 
    
	public DataParser() {
        readDataFromFile();
    }

    private void readDataFromFile() {

        try {
            BufferedReader reader = new BufferedReader(new FileReader(DATASET));
            
            String numberOfNodes = reader.readLine() ;
            graph = new EdgeWeightedDigraph(Integer.parseInt(numberOfNodes)) ;
            
            String line = null;

            while ((line = reader.readLine()) != null) {
                extractDirectedEdge(line);
            }
            reader.close();
            
        } catch (IOException | FileFormatException ex) {
			ex.printStackTrace();
        }
    }

    
    private void extractDirectedEdge(String line) throws FileFormatException {
        
    	String[] edgeInformation = line.split(" ");
    	
    	// Later change the name to String
        int node1 = Integer.parseInt(edgeInformation[0]);
        int node2 = Integer.parseInt(edgeInformation[1]);
        
        double weight = Double.parseDouble(edgeInformation[2]);

        
        DirectedEdge edge = new DirectedEdge(node1, node2, weight) ;
        graph.addEdge(edge);
    }
    
    public EdgeWeightedDigraph getGraph() {
    	return this.graph ;
    }
    
    public void setFilePath(String filepath) {
        this.DATASET = filepath;
    }
}
