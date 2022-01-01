package test;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import gumtreediff.actions.ActionGenerator;
import gumtreediff.actions.model.Action;
import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import split.Split;
import structure.SubTree;
import utils.Defuse;

public class DuplicateTest {

	public static void main(String args[]) throws Exception{
		String path = "talker.cpp";
		File cppfile = new File(path);
		TreeContext tc = new SrcmlCppTreeGenerator().generateFromFile(cppfile);
		ITree root = tc.getRoot();
		String path2 = "talker.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlCppTreeGenerator().generateFromFile(cppfile2);
        ITree root2 = tc2.getRoot();

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

        Split sp = new Split();
        Defuse defuse = new Defuse();
        ArrayList<SubTree> sub1 = sp.splitSubTree(tc, "test");//子树相似度测试
        ArrayList<SubTree> sub2 = sp.splitSubTree(tc2, "test1");//子树相似度测试
        for(SubTree st1 : sub1) {
        	ITree sRoot1 = st1.getRoot();
        	ITree sRoot2 = mappings.getDst(sRoot1);
        	SubTree st2 = null;
        	for(SubTree st : sub2) {
        		if(st.getRoot()==sRoot2) {
        			st2 = st;
        			break;
        		}
        	}
        	TreeContext newTC1 = defuse.buildTC(st1);
        	TreeContext newTC2 = defuse.buildTC(st2);
            Matcher m1 = Matchers.getInstance().getMatcher(newTC1.getRoot(), newTC2.getRoot());
            m1.match();
            MappingStore mappings1 = m1.getMappings();
            ActionGenerator g = new ActionGenerator(newTC1.getRoot(), newTC2.getRoot(), mappings1);
            List<Action> actions = g.generate();
            System.out.println("ACsize:"+actions.size());
        }
	}


}
