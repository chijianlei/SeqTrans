package test;

import gumtreediff.actions.ActionGenerator;
import gumtreediff.actions.model.*;
import gumtreediff.gen.srcml.SrcmlJavaTreeGenerator;
import gumtreediff.io.ActionsIoUtils;
import gumtreediff.io.TreeIoUtils;
import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import utils.Utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class TestGeneration {
	
	public static void main(String args[]) throws Exception{
//		String path = "astra_frame_listener.cpp";
		String path = "DefaultActionInvocation2.java";
//		String path = "migrations_test\\astra_driver\\astra_driver.cpp";
		File cppfile = new File(path);
		TreeContext tc = new SrcmlJavaTreeGenerator().generateFromFile(cppfile);
		ITree root = tc.getRoot();
		System.out.println(root.getId()+","+tc.getTypeLabel(root));
//		String path2 = "astra_frame_listener2.cpp";
		String path2 = "DefaultActionInvocation.java";
//		String path2 = "migrations_test\\astra_driver\\astra_driver2.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlJavaTreeGenerator().generateFromFile(cppfile2);
        ITree root2 = tc2.getRoot();
        System.out.println(root2.getId()+","+tc2.getTypeLabel(root2));
        
        System.out.println(tc.getRoot().getId());
        System.out.println(tc2.getRoot().getId());
        Matcher m = Matchers.getInstance().getMatcher(tc.getRoot(), tc2.getRoot());
        m.match();
        MappingStore mappings = m.getMappings();
        HashMap<Integer, Integer> mapping = new HashMap<>();       
        for(Mapping map : mappings) {
        	ITree src = map.getFirst();
        	ITree dst = map.getSecond();
//        	System.out.println("Mapping:"+src.getId()+"->"+dst.getId());
        	mapping.put(src.getId(), dst.getId());
        }
        System.out.println("mapSize:"+mapping.size());
        
        HashMap<Integer, Integer> mapping1 = new HashMap<>();       
        for(Mapping map : m.getMappings()) {
        	ITree src = map.getFirst();
        	ITree dst = map.getSecond();
//        	System.out.println("Mapping:"+src.getId()+"->"+dst.getId());
        	mapping1.put(src.getId(), dst.getId());
        }
        System.out.println("mapSize:"+mapping1.size());
        ActionGenerator g = new ActionGenerator(tc.getRoot(), tc2.getRoot(), m.getMappings());      
        List<Action> actions = g.generate();
        String out1 = "testMapping.txt";
        BufferedWriter wr1 = new BufferedWriter(new FileWriter(out1));
//        System.out.println(ActionsIoUtils.toJson(tc, g.getActions(), m.getMappings()).toString());
        wr1.append(ActionsIoUtils.toXml(tc, g.getActions(), m.getMappings()).toString()); 
        wr1.flush();
        wr1.close();   
        
//        ITree root3 = tc2.getRoot();
//        System.out.println(root3.getId()+","+tc2.getTypeLabel(root3));
//        if(root3.getParent()==null)
//        	System.out.println(true);
//        else System.out.println(false);

        Utils.checkTCRoot(tc);
        Utils.checkTCRoot(tc2);
//        Pruning pt = new Pruning(tc, tc2, mappings);
//        pt.pruneTree();//Prune the tree.       

        String out = "testGraph.txt";
        BufferedWriter wr = new BufferedWriter(new FileWriter(out));
        wr.append(TreeIoUtils.toDot(tc, mappings, actions, true).toString());
        wr.flush();
        wr.close();
        String out2 = "testGraph2.txt";
        BufferedWriter wr2 = new BufferedWriter(new FileWriter(out2));
        wr2.append(TreeIoUtils.toDot(tc2, mappings, actions, false).toString());
        wr2.flush();
        wr2.close(); 
        
//        System.out.println(TreeIoUtils.toDot(tc, mappings, actions, true).toString());
//        System.out.println(TreeIoUtils.toDot(tc2, mappings, actions, false).toString());
//        System.out.println(TreeIoUtils.toXml(tc).toString());    
//        for(Map.Entry<Integer, Integer> entry : mapping.entrySet()) {
//        	System.out.println(entry.getKey()+"->"+entry.getValue());
//        }
        
        System.out.println("ActionSize:" + actions.size());
        for (Action a : actions) {
            ITree src = a.getNode();
            if (a instanceof Move) {
                ITree dst = mappings.getDst(src);
                System.out.println(((Move)a).toString());
            } else if (a instanceof Update) {
                ITree dst = mappings.getDst(src);
                System.out.println(((Update)a).toString());
            } else if (a instanceof Insert) {
                ITree dst = a.getNode();
                System.out.println(((Insert)a).toString());
            } else if (a instanceof Delete) {
            	System.out.println(((Delete)a).toString());
            }
        }
        
//		System.out.println(ActionsIoUtils.toXml(tc, g.getActions(), m.getMappings()).toString());    
        
//		System.out.println(ActionsIoUtils.toText(tc, g.getActions(), m.getMappings()).toString());
//        for(ITree c : t.getChildren()) {
//        	if(c.getLabel()!=null)
//        		System.out.println(c.getLabel());
//        }
		List<ITree> nodes = new ArrayList<>();
		nodes = Utils.collectNode(tc2.getRoot(), nodes);
		System.out.println(nodes.size());

	}
		
		
		      

}
