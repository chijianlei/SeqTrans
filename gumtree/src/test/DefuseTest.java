package test;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

import gumtreediff.actions.model.Action;
import gumtreediff.gen.srcml.SrcmlJavaTreeGenerator;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import split.Split;
import structure.Definition;
import structure.SubTree;
import utils.Defuse;
import utils.Output;
import utils.Utils;

public class DefuseTest {

	public static void main (String args[]) throws Exception{
		String path = "migrations_test1";
		DefuseTest deftest = new DefuseTest();
		deftest.collectDiffwithDefUse(path, "", true, false, "");
	}

	public void collectDiffwithDefUse(String path, String outMode,
			Boolean ifOnlyChange, Boolean ifPrintDef, String filter) throws Exception {//获取DefUse
			Split sp = new Split();
			Defuse defuse = new Defuse();
			String path1 = "java_test\\ClassLoadingAwareObjectInputStream.java";
			String miName = path1;
			File javafile1 = new File(path1);
			TreeContext sTC = new SrcmlJavaTreeGenerator().generateFromFile(javafile1);
			String path2 = "java_test\\ClassLoadingAwareObjectInputStream2.java";
			File javafile2 = new File(path2);
			TreeContext dTC = new SrcmlJavaTreeGenerator().generateFromFile(javafile2);
			Matcher m = Matchers.getInstance().getMatcher(sTC.getRoot(), dTC.getRoot());
	        m.match();
			MappingStore mappings = m.getMappings();
			System.out.println("Analyse:"+miName);

			String jpath = "jsons\\";
			File jFile = new File(jpath);
			if(!jFile.exists())
				jFile.mkdirs();
			int count = 0;//计数
			if(jFile.listFiles().length!=0) {
				throw new Exception("Plz clean dir!");
			}
			File output = new File("defuse.txt");
			BufferedWriter wr = new BufferedWriter(new FileWriter(output));
			File output1 = new File("src-val.txt");
			File output2 = new File("tgt-val.txt");
			BufferedWriter wr1 = new BufferedWriter(new FileWriter(output1));
			BufferedWriter wr2 = new BufferedWriter(new FileWriter(output2));

			int errCount = 0;
			String tmpName = "";
			ArrayList<SubTree> changedSTree = new ArrayList<>();
			HashMap<String, LinkedList<Action>> actions = Utils.collectAction(sTC, dTC, mappings);
			ArrayList<Integer> srcActIds = Utils.collectSrcActNodeIds(sTC, dTC, mappings, actions);
			ArrayList<Definition> defs1 = defuse.getDef(sTC, "src");//先计算action,再收集defs
	        ArrayList<Definition> defs2 = defuse.getDef(dTC, "tgt");
	        HashMap<String, ArrayList<Definition>> defMap1 = defuse.transferDefs(defs1);
	        HashMap<String, ArrayList<Definition>> defMap2 = defuse.transferDefs(defs2);
	        HashMap<ITree, ArrayList<Definition>> blockMap1 = defuse.transferBlockMap(defs1, sTC, "src");
	        HashMap<ITree, ArrayList<Definition>> blockMap2 = defuse.transferBlockMap(defs2, dTC, "tgt");
			ArrayList<SubTree> sub1 = sp.splitSubTree(sTC, miName);//Subtree中割裂过block,注意
			ArrayList<SubTree> sub2 = sp.splitSubTree(dTC, miName);//先计算action,再split ST

			System.out.println(defs1.size());
	        System.out.println(defs2.size());
	        System.out.println(defMap1.size());
	        System.out.println(defMap2.size());
	        System.out.println(blockMap1.size());
	        System.out.println(blockMap2.size());

	        for(SubTree st : sub1) {
	        	ITree root = st.getRoot();
	        	System.out.println("StID:"+root.getId());
	        	if(root.getId()==822) {
	        		List<ITree> des = root.getDescendants();
	        		for(ITree node : des) {
	        			System.out.println(node.getId()+","+sTC.getTypeLabel(node)+","+node.getLabel());
	        		}
	        	}
	        }
	        for(SubTree st : sub2) {
	        	ITree root = st.getRoot();
//	        	System.out.println("StID:"+root.getId());
	        	if(root.getId()==986) {
	        		List<ITree> des = root.getDescendants();
	        		for(ITree node : des) {
	        			System.out.println(node.getId()+","+dTC.getTypeLabel(node)+","+node.getLabel());
	        		}
	        	}
	        }
	        if(ifOnlyChange) {
				for(SubTree st : sub1) {
					ITree t = st.getRoot();
					List<ITree> nodeList = t.getDescendants();
					nodeList.add(t);
		        	for(ITree node : nodeList) {
		        		int id = node.getId();
		        		if(srcActIds.contains(id)) {
		        			changedSTree.add(st);
//		        			System.out.println("find a action subtree!");
		        			if(t.getId()==2256) {
		        				System.err.println("action id:"+id);
		        			}
		        			break;
		        		}
		        	}
				}//先找包含action的subtree
	        }else {
	        	changedSTree = sub1;
	        }

	        System.out.println("subSize:"+sub1.size());
			System.out.println("changeSize:"+changedSTree.size());
			for(SubTree srcT : changedSTree) {
				System.out.println("===================");
				HashSet<Definition> usedDefs1 = new HashSet<>();
				HashSet<Definition> usedDefs2 = new HashSet<>();
				ITree sRoot = srcT.getRoot();
				System.out.println("CheckMapping "+sRoot.getId()+":"+srcT.getMiName());
				String src = Output.subtree2src(srcT);

	    		if(outMode=="txt") {
	    			if(src.contains("error")&&src.contains("situation")) {
	    				errCount++;
	    				continue;
	    			}
	    		}

	    		Boolean same = false;
				ArrayList<ITree> leaves1 = new ArrayList<>();
				Utils.traverse2Leaf(sRoot, leaves1);
				int labelCount = 0;
				for(ITree leaf : leaves1) {
					String label = leaf.getLabel();
					System.out.println("label:"+label);
					if(!label.equals(""))
						labelCount++;
					String type = sTC.getTypeLabel(leaf);
					if(type.equals("literal")) {
						leaf.setLabel(Output.deleteLiteral(leaf, sTC));
					}
					ArrayList<Definition> stringList = defMap1.get(label);
					if(stringList!=null) {
						ITree parBlock = defuse.searchBlock(leaf, sTC);
						ArrayList<Definition> blockList = blockMap1.get(parBlock);
						for(Definition def1 : stringList) {
							if(blockList!=null) {
								if(blockList.contains(def1)) {
									if(leaf.getId()>def1.getDefLabelID()) {
										leaf.setLabel("var");
										usedDefs1.add(def1);
									}
								}
							}
							if(def1.getDefLabelID()==leaf.getId()) {
								leaf.setLabel("var");
							}
							System.out.println(leaf.getId()+","+def1.getDefLabelID());
							System.out.println("Def:"+def1.getType()+","+def1.getVarName());
						}
					}
				}
				if(labelCount<=0)
					continue;

				SubTree dstT = defuse.checkMapping(srcT, mappings, dTC, sub2);
				if(dstT==null)
					continue;//子树没有对应子树，被删除
				ITree dRoot = dstT.getRoot();
	    		System.out.println(sRoot.getId()+"->"+dRoot.getId());

				ArrayList<ITree> leaves2 = new ArrayList<>();
				Utils.traverse2Leaf(dRoot, leaves2);
				for(ITree leaf : leaves2) {
					String label = leaf.getLabel();
					String type = dTC.getTypeLabel(leaf);
					if(type.equals("literal")) {
						leaf.setLabel(Output.deleteLiteral(leaf, dTC));
					}
					if(dRoot.getId()==226) {
						System.err.println(label);
					}
					ArrayList<Definition> stringList = defMap2.get(label);
					if(stringList!=null) {
						ITree parBlock = defuse.searchBlock(leaf, dTC);
						ArrayList<Definition> blockList = blockMap2.get(parBlock);
						if(dRoot.getId()==226) {
							System.err.println(blockList.size());
						}
						for(Definition def2 : stringList) {
							if(blockList!=null) {
								if(blockList.contains(def2)) {
									if(leaf.getId()>def2.getDefLabelID()) {
										usedDefs2.add(def2);
										leaf.setLabel("var");
									}
									System.out.println(leaf.getId()+","+def2.getDefLabelID());
									System.out.println(def2.getType()+","+def2.getVarName());
								}
							}
							if(def2.getDefLabelID()==leaf.getId()) {
								leaf.setLabel("var");
							}
						}
					}
					if(!same) {
						for(ITree leaf1 : leaves1) {
							String label1 = leaf1.getLabel();
							if(label.equals(label1)) {
								same = true;
							}
						}
					}
				}
				src = Output.subtree2src(srcT);
	    		String tar = Output.subtree2src(dstT);
	    		if(outMode=="txt") {
		    		if(tar.contains("error")&&tar.contains("situation")) {
		    			errCount++;
		    			continue;
		    		}
	    		}
	    		if(ifOnlyChange) {
	    			if(src.equals(tar))
		    			continue;
	    		}

	    		//长度相差太多的句子直接跳过
				if(((float)src.length()/(float)tar.length())<0.25||((float)tar.length()/(float)src.length())<0.25 || !same)
					continue;//no leaf is the same
				if(outMode=="txt") {
					if(ifPrintDef) {
						String buffer = "";
						String buffer1 = "";
						String buffer2 = "";
						for(Definition def : usedDefs1) {
							SubTree st = new SubTree(def.getRoot(), sTC, 0, "");
							String stat = Output.subtree2src(st);
							buffer = buffer +stat+" ; ";
							buffer1 = buffer1 +stat+" ; ";
						}
						src = Output.subtree2src(srcT);
//						src += String.valueOf(srcT.getRoot().getId());
						buffer = buffer + src+"\t";
						buffer1 += src;
						for(Definition def : usedDefs2) {
							SubTree st = new SubTree(def.getRoot(), dTC, 0, "");
							String stat = Output.subtree2src(st);
							buffer += stat+" ; ";
							buffer2 += stat+" ; ";
						}
						tar = Output.subtree2src(dstT);
//						tar += String.valueOf(dstT.getRoot().getId());
						buffer += tar;
						buffer2 += tar;
						if(buffer.contains("error")&&buffer.contains("situation"))
							continue;
						wr.append(buffer);
						wr1.append(buffer1);
						wr2.append(buffer2);
						wr1.newLine();
						wr1.flush();
						wr2.newLine();
						wr2.flush();
						wr.newLine();
						wr.flush();
					}else {
						src += String.valueOf(srcT.getRoot().getId());
						tar += String.valueOf(dstT.getRoot().getId());
			    		String buffer = src+"\t"+tar;
						if(buffer.contains("error")&&buffer.contains("situation"))
							continue;
						wr.append(buffer);
						wr1.append(src);
						wr2.append(tar);
						wr1.newLine();
						wr1.flush();
						wr2.newLine();
						wr2.flush();
						wr.newLine();
						wr.flush();
					}
				}else if(outMode=="json") {
					TreeContext st = defuse.buildTC(srcT);
					TreeContext dt = defuse.buildTC(dstT);
					defuse.printJson(jpath, count, st, dt);
					count++;
				}
			}
			wr.close();
			wr1.close();
			wr2.close();
			System.out.println("miname:"+tmpName);
			System.out.println("errCount:"+errCount);
		}

}
