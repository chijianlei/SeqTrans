package utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import gumtreediff.tree.TreeUtils;
import structure.Edge;

public class Printing {

	public static void main (String args[]) throws Exception{
		String path1 = "talker.cpp";
		File cppfile1 = new File(path1);
		TreeContext tc1 = new SrcmlCppTreeGenerator().generateFromFile(cppfile1);
		String path2 = "talker2.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlCppTreeGenerator().generateFromFile(cppfile2);
		mergeGraph(tc1, tc2);
	}

	public static void mergeGraph(TreeContext tc1, TreeContext tc2) throws IOException {
        Matcher m = Matchers.getInstance().getMatcher("gumtree-topdown", tc1.getRoot(), tc2.getRoot());
        m.match();//只包含topdown找到的同构子树mapping
        MappingStore mappings = m.getMappings();
        ArrayList<ITree> mapNodes1 = new ArrayList<>();
        ArrayList<ITree> mapNodes2 = new ArrayList<>();
        ArrayList<Edge> edges = new ArrayList<>();
        HashMap<Integer, ITree> idMap1 = new HashMap<>();
        HashMap<Integer, ITree> idMap2 = new HashMap<>();
        HashMap<Integer, Integer> newid_map2 = new HashMap<>();
        for(Mapping map : mappings) {
        	ITree src = map.getFirst();
        	ITree dst = map.getSecond();
        	System.out.println("Mapping:"+src.getId()+"->"+dst.getId());
        	mapNodes1.add(src);
        	mapNodes2.add(dst);
        }

//        ArrayList<ITree> subRoots1 = searchSubRoot(mapNodes1);//找同构子树根节点
        ArrayList<ITree> subRoots1 = mapNodes1;//所有同构子树节点全部包括

        BufferedWriter wr = new BufferedWriter(new FileWriter(new File("tokens.txt")));
        BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File("edges.txt")));
        BufferedWriter wr2 = new BufferedWriter(new FileWriter(new File("merge_nodes.txt")));
        BufferedWriter wr3 = new BufferedWriter(new FileWriter(new File("node_map.txt")));
        ITree root1 = tc1.getRoot();
		List<ITree> preOrder1 = TreeUtils.preOrder(root1);
		int nodeNum = preOrder1.size();
		for(ITree node : preOrder1) {
			int id = node.getId();
			idMap1.put(id, node);
		}
		ITree root2 = tc2.getRoot();
		int nextID = nodeNum;
		List<ITree> preOrder2 = TreeUtils.preOrder(root2);
		for(ITree node : preOrder2) {
			int id = node.getId();
			idMap2.put(id, node);
		}//put AST nodes into ordered map

		for(Map.Entry<Integer, ITree> entry : idMap2.entrySet()) {
			int id = entry.getKey();
			ITree node = entry.getValue();
			int oldId = id;
			if(mapNodes2.contains(node)) {
				id = mappings.getSrc(node).getId();//将dst的id转成src的id
				System.out.println(id+" "+oldId);
				wr2.append(id+" "+oldId);
				wr2.newLine();
				wr2.flush();
				newid_map2.put(oldId, id);
			}else {
				id = nextID;
				newid_map2.put(oldId, id);
				nextID++;
			}
		}// traverse tc2 firstly and put oldid and newid into the map

		for(Map.Entry<Integer, ITree> entry : idMap1.entrySet()) {
			int id = entry.getKey();
			ITree node = entry.getValue();
			ITree par = node.getParent();
			String type = tc1.getTypeLabel(node);
			String label = "NULL";
			if(node.hasLabel()) {
				label = node.getLabel();
			}
			if(par!=null) {
				Edge edge = new Edge(par.getId(), id);
				if(!edges.contains(edge)) {
					wr1.append(par.getId()+" "+id);
					wr1.newLine();
					wr1.flush();
					edges.add(edge);
				}
			}
			wr.append(String.valueOf(id)+","+type+","+label);
			wr.newLine();
			wr.flush();
		}

		for(Map.Entry<Integer, ITree> entry : idMap2.entrySet()) {
			int id = entry.getKey();
			ITree node = entry.getValue();
			ITree par = node.getParent();
			String type = tc2.getTypeLabel(node);
			String label = "NULL";
			if(node.hasLabel()) {
				label = node.getLabel();
			}
			int oldId = id;
//			id += nodeNum;//将TC2的id加在TC1之后
			System.out.println(id+","+newid_map2.get(id));
			id = newid_map2.get(id);

			wr3.append(oldId+"->"+id);
			wr3.newLine();
			wr3.flush();
//			System.out.println(id);
			if(par!=null) {
				int parID = par.getId();
				parID = newid_map2.get(parID);
				Edge edge = new Edge(parID, id);
				if(!edges.contains(edge)) {
					wr1.append(parID+" "+id);
					wr1.newLine();
					wr1.flush();
					edges.add(edge);
				}
			}
			if(!mapNodes2.contains(node)) {
				wr.append(String.valueOf(id)+","+type+","+label);
				wr.newLine();
				wr.flush();
			}
		}

//		int vitualID = nodeNum+preOrder2.size();
//		for(ITree src : subRoots1) {
//			ITree dst = mappings.getDst(src);
//			int srcID = src.getId();
//			int dstID = nodeNum+dst.getId();
//			wr1.append(srcID+" "+vitualID);
//			wr.append(vitualID+",virtualNode,NULL");
//			wr.newLine();
//			wr.flush();
//			wr2.append(srcID+","+dstID);
//			wr2.newLine();
//			wr2.append(String.valueOf(vitualID));
//			wr1.newLine();
//			wr2.newLine();
//			wr1.append(dstID+" "+vitualID);
//			wr1.newLine();
//			wr1.flush();
//			wr2.flush();
//			vitualID++;
//		}
		wr.close();
		wr1.close();
		wr2.close();
		wr3.close();
	}

	private static ArrayList<ITree> searchSubRoot(ArrayList<ITree> mapNodes) {
		ArrayList<ITree> subRoots = new ArrayList<>();
        for(ITree node : mapNodes) {
        	boolean hasPar = false;
        	List<ITree> pars = node.getParents();
        	for(ITree node1 : mapNodes) {
        		if(pars.contains(node1)) {
        			hasPar = true;
        			break;
        		}
        	}
        	if (!hasPar) {
				subRoots.add(node);
			}
        }
        return subRoots;
	}

}
