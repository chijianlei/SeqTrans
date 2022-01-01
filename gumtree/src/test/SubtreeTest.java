package test;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.gen.srcml.SrcmlJavaTreeGenerator;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import gumtreediff.tree.TreeUtils;
import split.Split;
import structure.SubTree;
import utils.Similarity;
import utils.Utils;

public class SubtreeTest {

	public static void main (String args[]) throws Exception{
		String path1 = "java_test\\BZip2CompressorOutputStream.java";
		File cppfile = new File(path1);
		TreeContext tc1 = new SrcmlJavaTreeGenerator().generateFromFile(cppfile);
		ITree root1 = tc1.getRoot();
		List<ITree> des1 = root1.getDescendants();
		String miName = path1.split("/")[path1.split("/").length-1];//标记文件名
		String path2 = "java_test\\BZip2CompressorOutputStream2.java";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlJavaTreeGenerator().generateFromFile(cppfile2);
		Split sp = new Split();
		Matcher m = Matchers.getInstance().getMatcher(tc1.getRoot(), tc2.getRoot());
        m.match();
        MappingStore mappings = m.getMappings();
        ArrayList<SubTree> sts1 = sp.splitSubTree(tc1, miName);
        ArrayList<SubTree> sts2 = sp.splitSubTree(tc2, miName);

        String search_string = "blockSort";
        String[] tmps = search_string.split(" ");
        List<String> labels = Arrays.asList(tmps);
        searchSubtree(labels, sts1);
//        ITree sRoot = searchNode(69, des1);
//        ITree dRoot = mappings.getDst(sRoot);
//        System.out.println(dRoot.getId());
//        BufferedWriter wr = new BufferedWriter(new FileWriter(new File("mapping.txt")));
//        for(Mapping map : mappings) {
//        	ITree src = map.getFirst();
//        	ITree dst = map.getSecond();
//        	wr.append(src.getId()+"->"+dst.getId());
//        	wr.newLine();
////        	System.out.println("Mapping:"+src.getId()+"->"+dst.getId());
////        	if(dst.getId()==553) {
////        		System.err.println("find it!"+src.getId());
////        	}
//        }
//        wr.close();
//        System.out.println("Msize:"+mappings.asSet().size());
//        for(SubTree st : sts1) {
//        	ITree root = st.getRoot();
//        	System.out.println("StID:"+root.getId()+":"+root.getLine()+","+root.getColumn()+
//        			"->"+root.getLastLine()+","+root.getLastColumn()+" Len:"+root.getLength());
//    		List<ITree> des = root.getDescendants();
//        	for(ITree node : des) {
//        		System.out.println(node.getId()+":"+node.getLine()+","+node.getColumn()+
//        				"->"+node.getLastLine()+","+node.getLastColumn()+" Len:"+node.getLength());
//        	}
////        	if(root.getId()==41) {
////        		for(ITree node : root.postOrder()) {
////        			System.out.println("ID:"+node.getId());
////        		}
////        		List<ITree> des = root.getDescendants();
////            	for(ITree node : des) {
////            		System.out.println(node.getId());
////            		ITree dst = mappings.getDst(node);
////            		if(dst!=null)
////            		System.out.println(node.getId()+"->"+dst.getId());
////            	}
////        	}
//        }
//        System.out.println("-----");
//        for(SubTree st : sts2) {
//        	ITree root = st.getRoot();
//        	String dst = Output.subtree2src(st);
////        	System.out.println(dst);
//        }
	}

	public static ITree searchNode(int id, List<ITree> des) {
		for(ITree node : des) {
			int nodeID = node.getId();
			if(id==nodeID)
				return node;
		}
		return null;
	}

	public static void searchSubtree(List<String> labels, ArrayList<SubTree> sts) throws Exception {
		for(SubTree st : sts) {
			float matchNum = 0;
			ITree sRoot = st.getRoot();
			List<ITree> leaves = new ArrayList<>();
			String print = "";
			leaves = Utils.traverse2Leaf(sRoot, leaves);
			for(ITree leaf : leaves) {
				String label = leaf.getLabel();
				if (labels.contains(label)) {
					matchNum++;
				}
			}
			float sim = matchNum/leaves.size();
			if(sim>0.9) {
				System.err.println("Find matched subtree: "+sRoot.getId());
				for(ITree leaf : leaves) {
					print += leaf.getLabel()+" ";
				}
				System.out.println(print);
			}

		}
	}

	public static void breakBlock() throws Exception {
		String path = "talker.cpp";
		File cppfile = new File(path);
		TreeContext tc1 = new SrcmlCppTreeGenerator().generateFromFile(cppfile);
		Split sp = new Split();
		ArrayList<SubTree> sts = sp.splitSubTree(tc1, path);
		System.out.println(sts.size());
		for(SubTree st : sts) {
			ITree sRoot = st.getRoot();
			if(tc1.getTypeLabel(sRoot).equals("while")) {
				String srcTree = Similarity.transfer2string(st);
				System.out.println(srcTree);
				List<ITree> list = TreeUtils.preOrder(st.getRoot());
				for(ITree tmp : list) {
					String type = tc1.getTypeLabel(tmp);
					if(type.equals("block")) {
						List<ITree> childs = sRoot.getChildren();
						System.out.println(childs.size());
						childs.remove(tmp);
						System.out.println(sRoot.getChildren().size());
						sRoot.setChildren(childs);
						tmp.setParent(null);//断开所有block node和父亲的连接
					}
				}
				String srcTree1 = Similarity.transfer2string(st);
				System.out.println(srcTree1);
			}
		}
	}

}
