package collect;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import gumtreediff.gen.srcml.SrcmlJavaTreeGenerator;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import split.Split;
import structure.Definition;
import structure.Location;
import structure.Migration;
import structure.SubTree;
import utils.Defuse;
import utils.Output;
import utils.Utils;

public class CollectSingleFile {
	
	public static void main(String[] args) throws Exception {
		List<String> parList = Arrays.asList(args);
		String inputPath = null;
		String location = null;
		String outputPath = null;
//		if(args.length==0)
//			throw new Exception("Plz input file and locations");
//		if(!parList.contains("--input"))
//			throw new Exception("no input");
//		if(!parList.contains("--location"))
//			throw new Exception("no location");
//		if(!parList.contains("--location")) {
//			Path path = Paths.get("");
//			String directoryName = path.toAbsolutePath().toString();
//			outputPath = directoryName;
//		}
//			
//
//		for(int i=0;i<parList.size();i++) {
//			String par = parList.get(i);
//			if(par.equals("--input")) {
//				String nextPar = parList.get(i+1);
//				inputPath = nextPar;
//			}
//			if(par.equals("--location")) {
//				String nextPar = parList.get(i+1);
//				location = nextPar;
//			}	
//			if(par.equals("--output")) {
//				String nextPar = parList.get(i+1);
//				outputPath = nextPar;
//			}			
//		}
		
		inputPath = "D:\\workspace\\eclipse2018\\Migration\\AccountInstance.java";
		location = String.valueOf(15);
		outputPath = "D:\\workspace\\eclipse2018\\Migration\\test";
		filterDefUse(inputPath, Integer.valueOf(location), outputPath);
	}
	
	public static void filterDefUse(String input, int location, String output) throws Exception {
		File inputFile = new File(input);
		String var_path = output+File.separator+"src-val.txt";
		String num_path = output+File.separator+"src-num.txt";
		String check_path = output+File.separator+"src_check.txt";
		TreeContext tc = new SrcmlJavaTreeGenerator().generateFromFile(inputFile);
		ITree root = tc.getRoot();		
		List<ITree> all_nodes = root.getDescendants();
		all_nodes.add(root);
		System.out.println(all_nodes.size());
		ArrayList<ITree> locateNodes = new ArrayList<>();
		for(ITree node : all_nodes) {
			int start_line = node.getLine();
			if(start_line==location) {
				locateNodes.add(node);
			}
		}
		if(locateNodes.size()==0)
			throw new Exception("plz check the location!");
		
		System.out.println("Analyse: "+input);		
		Split sp = new Split();	
		Defuse defuse = new Defuse();	
		ArrayList<Definition> defs = defuse.getDef(tc, "src");//先计算action,再收集defs
		HashMap<String, ArrayList<Definition>> defMap = defuse.transferDefs(defs);
		HashMap<ITree, ITree> leaf2parblock_map = defuse.searchBlockMap(tc);
        HashMap<ITree, ArrayList<Definition>> blockMap = defuse.transferBlockMap(defs, tc, "src");       
        ArrayList<SubTree> sub = sp.splitSubTree(tc, input);//Subtree中割裂过block,注意
		HashMap<Integer, HashMap<String, String>> usedDefs2Map = new HashMap<Integer, HashMap<String, String>>();
		System.out.println("def1size:"+defs.size());
        System.out.println("def1Mapsize:"+defMap.size());
        System.out.println("block1size:"+blockMap.size());
        
        ArrayList<SubTree> changedSTree = new ArrayList<>();
        for(SubTree st : sub) {
			ITree t = st.getRoot();
//			System.out.println("StID:"+t.getId());
			List<ITree> nodeList = t.getDescendants();
			nodeList.add(t);
        	for(ITree node : nodeList) {	        		
        		if(locateNodes.contains(node)) {
        			changedSTree.add(st);
//        			System.out.println("find a action subtree! "+t.getId());
        			break;
        		}
        	}
		}//先找包含action的subtree
        
        System.out.println("subSize:"+sub.size());	
		System.out.println("changeSize:"+changedSTree.size());	
		for(SubTree st : changedSTree) {	
			ITree sRoot = st.getRoot();
			int id = sRoot.getId();				
			System.out.println("===================");
			System.out.println("StID:"+id);
			HashMap<String, String> replaceMap_src = new HashMap<String, String>();
			HashSet<Definition> usedDefs1 = new HashSet<Definition>();										
    			
    		Boolean same = false;
			ArrayList<ITree> leaves1 = new ArrayList<ITree>();				
			Utils.traverse2Leaf(sRoot, leaves1);
			int labelCount = 0;
			for(ITree leaf : leaves1) {
				String label = leaf.getLabel();
//				System.out.println("label:"+label);
				if(!label.equals(""))
					labelCount++;
				String type = tc.getTypeLabel(leaf);
				if(type.equals("literal")) {
					leaf.setLabel(Output.deleteLiteral(leaf, tc));
//					if(label.contains("\"")) 
//						replaceMap_src.put("@@"+label+"@@", "None");
//					else
//						replaceMap_src.put("$$"+label+"$$", "num");//replace Literal
				}					
				ArrayList<Definition> stringList = defMap.get(label);
				if(stringList!=null) {
					ITree parBlock = leaf2parblock_map.get(leaf);
					ArrayList<Definition> blockList = blockMap.get(parBlock);
					for(Definition def1 : stringList) {
						if(blockList!=null) {
							if(blockList.contains(def1)) {
								if(leaf.getId()>def1.getDefLabelID()) {
									usedDefs1.add(def1);
									System.out.println("DefTest: "+leaf.getLabel()+","+leaf.getId()+","+def1.getDefLabelID());
//									leaf.setLabel("var");										
									replaceMap_src.put(label, label);
								}											
							}
						}							
						if(def1.getDefLabelID()==leaf.getId()) {
//							leaf.setLabel("var");
							replaceMap_src.put(label, label);
						}
//						System.out.println(leaf.getId()+","+def1.getDefLabelID());
//						System.out.println("Def:"+def1.getType()+","+def1.getVarName());
					}
				}
			}
			if(labelCount==0) {
				System.err.println("labelCount is 0 ID:"+id);
				continue;
			}								
			Location location1 = new Location(st);   														    		
				
			String diffLine_check = "STID:"+st.getRoot().getId()+","
					+location1.getBeginLine()+","+location1.getLastLine()+","
					+location1.getBeginCol()+","+location1.getLastCol();						
			String diffLine = input+";"
					+location1.getBeginLine()+","+location1.getLastLine()+","
					+location1.getBeginCol()+","+location1.getLastCol();							
			printLineNum(num_path, diffLine);
			printDefs(var_path, replaceMap_src, usedDefs1);
			printLineCheck(check_path, diffLine_check);
		}
	}
	
	static private void printLineCheck(String outPath6, String diffLine_check) throws IOException {
		File output6 = new File(outPath6);
		BufferedWriter wr6 = new BufferedWriter(new FileWriter(output6, true));
		wr6.append(diffLine_check);
		wr6.newLine();
		wr6.flush();
		wr6.close();
	}
	
	static private void printLineNum(String outPath3, String diffLine) throws Exception {
		File output3 = new File(outPath3);
		BufferedWriter wr3 = new BufferedWriter(new FileWriter(output3, true));
        wr3.append(diffLine);
		wr3.newLine();
		wr3.flush();
//		System.out.println("STID:"+srcT.getRoot().getId()+","+dstT.getRoot().getId());
//		System.out.println(replaceMap_dst.size());
		wr3.close();
	}
	
	static private void printDefs(String outPath4, HashMap<String , String> replaceMap_src,
			HashSet<Definition> usedDefs1) throws IOException {		
		File output4 = new File(outPath4);
		BufferedWriter wr4 = new BufferedWriter(new FileWriter(output4, true));
		
		for(Definition def1 : usedDefs1) {
			SubTree st1 = new SubTree(def1.getRoot(), def1.getTc(), 0, "");
			Location location1 = new Location(st1);
			wr4.append(+location1.getBeginLine()+","+location1.getLastLine()
			+","+location1.getBeginCol()+","+location1.getLastCol()+";");
		}
		
		for(Map.Entry<String, String> entry : replaceMap_src.entrySet()) {
			String varName = entry.getKey();
			String label = entry.getValue();
			wr4.append(varName+"->"+label+";");
		}
		wr4.newLine();
		wr4.flush();
		wr4.close();
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

}
