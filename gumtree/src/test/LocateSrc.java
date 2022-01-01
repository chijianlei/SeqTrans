package test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import gumtreediff.actions.model.Action;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import split.Split;
import structure.Migration;
import structure.SubTree;
import utils.Output;
import utils.Utils;

public class LocateSrc {

	public static void main (String args[]) throws Exception{
		String path = "tmp\\output";
		String src = "var = boost :: make_shared < du :: Updater >  ( var , var , getName  )";
		String tgt = "this -> set_on_parameters_set_callback ( std :: bind ( & FeedforwardPid :: ReconfigCb , this , std :: placeholders :: _1 )  FeedforwardPid :: ReconfigCb , this , std :: placeholders :: _1 )";
		locateSrc(path, src, tgt);
	}

	public static void locateSrc(String path, String srcString, String tgtString) throws Exception {
		Split sp = new Split();
		ArrayList<Migration> migrats = sp.readMigration(path, "");
		for(Migration migrat : migrats) {
			HashMap<String, ArrayList<Integer>> sDefMap = new HashMap<>();
			HashMap<String, ArrayList<Integer>> dDefMap = new HashMap<>();
			HashMap<Integer, SubTree> stMap = new HashMap<>();
			HashMap<Integer, SubTree> dtMap = new HashMap<>();
			String miName = migrat.getMiName();
			TreeContext sTC = migrat.getSrcT();
			TreeContext dTC = migrat.getDstT();
			MappingStore mappings = migrat.getMappings();

			System.out.println("Analyse:"+miName);
			Matcher m = Matchers.getInstance().getMatcher(sTC.getRoot(), dTC.getRoot());
	        m.match();
			ArrayList<SubTree> changedSTree = new ArrayList<>();
			HashMap<String, LinkedList<Action>> actions = Utils.collectAction(sTC, dTC, mappings);
			ArrayList<Integer> srcActIds = Utils.collectSrcActNodeIds(sTC, dTC, mappings, actions);
			ArrayList<SubTree> sub1 = sp.splitSubTree(sTC, miName);//Subtree中割裂过block,注意
			ArrayList<SubTree> sub2 = sp.splitSubTree(dTC, miName);//先计算action,再split ST

			for(SubTree st : sub1) {
				ITree t = st.getRoot();
				List<ITree> nodeList = t.getDescendants();
				nodeList.add(t);
	        	for(ITree node : nodeList) {
	        		int id = node.getId();
	        		if(srcActIds.contains(id)) {
	        			changedSTree.add(st);
//	        			System.out.println("find a action subtree!");
	        			break;
	        		}
	        	}
			}//先找包含action的subtree

			for(SubTree st : sub1) {
				ITree sRoot = st.getRoot();
				String sType = sTC.getTypeLabel(sRoot);
				if(sType.equals("decl_stmt")) {
					List<ITree> children = sRoot.getChildren();
					for(ITree root : children) {
						if(sTC.getTypeLabel(root).equals("decl")) {
							List<ITree> childs = root.getChildren();
							for(ITree child : childs) {
								String type = sTC.getTypeLabel(child);
								if(type.equals("name")) {//只有decl下的name节点的value才被认为是一条def
									String label = child.getLabel();
									int sID = sRoot.getId();
									stMap.put(sID, st);
									if(sDefMap.get(label)==null) {
										ArrayList<Integer> ids = new ArrayList<>();
										ids.add(sID);
										sDefMap.put(label, ids);
									}else {
										sDefMap.get(label).add(sID);
									}
								}
							}
						}else {
							throw new Exception("error type!"+sTC.getTypeLabel(root));
						}
					}
				}
			}
			for(SubTree dt : sub2) {
				ITree dRoot = dt.getRoot();
				String dType = sTC.getTypeLabel(dRoot);
				if(dType.equals("decl_stmt")) {
					List<ITree> children = dRoot.getChildren();
					for(ITree root : children) {
						if(sTC.getTypeLabel(root).equals("decl")) {
							List<ITree> childs = root.getChildren();
							for(ITree child : childs) {
								String type = dTC.getTypeLabel(child);
								if(type.equals("name")) {//只有decl下的name节点的value才被认为是一条def
									String label = child.getLabel();
									int dID = dRoot.getId();
									dtMap.put(dID, dt);
									if(dDefMap.get(label)==null) {
										ArrayList<Integer> ids = new ArrayList<>();
										ids.add(dID);
										dDefMap.put(label, ids);
									}else {
										dDefMap.get(label).add(dID);
									}
								}
							}
						}
					}
				}
			}

			for(SubTree srcT : changedSTree) {
				HashMap<String, SubTree> sMap = new HashMap<>();
				HashMap<String, SubTree> dMap = new HashMap<>();
				ArrayList<SubTree> sDef = new ArrayList<>();
				ArrayList<SubTree> dDef = new ArrayList<>();
				ArrayList<String> commonDef = new ArrayList<>();//change pair的def交集
				HashMap<String, String> varMap = new HashMap<>();//change pair的def总集
				ITree sRoot = srcT.getRoot();
	    		ITree dRoot = mappings.getDst(sRoot);
	    		SubTree dstT = null;
	    		if(dRoot==null) {
	    			continue;
	    		}else {//根据mapping来找dt
	    			for(SubTree dt : sub2) {
	    				ITree root = dt.getRoot();
	    				if(root.equals(dRoot)) {
	    					dstT = dt;
	                		break;
	    				}
	    			}
	    		}
	    		if(dstT==null) {
	    			throw new Exception("why is null?");
	    		}

	    		String src = Output.subtree2src(srcT);
	    		String tar = Output.subtree2src(dstT);
	    		if((src.contains("error")&&src.contains("situation")) || (tar.contains("error")&&tar.contains("situation")))
	    			continue;
	    		if(((float)src.length()/(float)tar.length())<0.25||((float)tar.length()/(float)src.length())<0.25) {
	    			continue;
	    		}//长度相差太多的句子直接跳过

				String sType = sTC.getTypeLabel(sRoot);
				String dType = dTC.getTypeLabel(dRoot);
				if(!sType.equals("decl_stmt")&&!dType.equals("decl_stmt")) {
					List<ITree> sLeaves = new ArrayList<>();
					Utils.traverse2Leaf(sRoot, sLeaves);
					List<ITree> dLeaves = new ArrayList<>();
					Utils.traverse2Leaf(dRoot, dLeaves);
					for(ITree leaf : sLeaves) {
						String type = sTC.getTypeLabel(leaf);
						if(type.equals("name")) {
							String label = leaf.getLabel();
							ArrayList<Integer> map = sDefMap.get(label);
							if(map!=null) {
								if(map.size()==1) {
									int defID = map.get(0);
									SubTree st = stMap.get(defID);
									sMap.put(label, st);
								}else {//发现有多个def line同一个关键字的情况，可能发生在不同的method
									Collections.sort(map);
									ArrayList<Integer> subMap = new ArrayList<>();
									for(int id : map) {//只取该条语句之前的subtree,抛弃掉之后的
										if(id<sRoot.getId())
											subMap.add(id);
									}
									if(subMap.size()==0)
										continue;//好像有defid全在rootid之后的情况，跳过
									int defID = subMap.get(subMap.size()-1);//取离该def最近的
									SubTree st = stMap.get(defID);
									sMap.put(label, st);
								}
							}
						}
					}
					for(ITree leaf : dLeaves) {
						String type = dTC.getTypeLabel(leaf);
						if(type.equals("name")) {
							String label = leaf.getLabel();
							ArrayList<Integer> map = dDefMap.get(label);
							if(map!=null) {
								if(map.size()==1) {
									int defID = map.get(0);
									SubTree st = dtMap.get(defID);
									dMap.put(label, st);
								}else {
									Collections.sort(map);
									ArrayList<Integer> subMap = new ArrayList<>();
									for(int id : map) {//只取该条语句之前的subtree,抛弃掉之后的
										if(id<dRoot.getId())
											subMap.add(id);
									}
									if(subMap.size()==0)
										continue;//好像有defid全在rootid之后的情况，跳过
									int defID = subMap.get(subMap.size()-1);//取离该def最近的
									SubTree st = dtMap.get(defID);
									dMap.put(label, st);
								}
							}
						}
					}
					if(sMap.size()>dMap.size()) {
						for(Map.Entry<String, SubTree> entry : dMap.entrySet()) {
							String keyword = entry.getKey();
							SubTree dt = entry.getValue();
							if(sMap.containsKey(keyword)) {//sMap和dMap取keyword交集
								sDef.add(sMap.get(keyword));
								dDef.add(dt);
								commonDef.add(keyword);//共有的keyword放到commonDef
								String absVarName = "var";
								varMap.put(keyword, absVarName);//共有的keyword放到varMap
							}
						}
					}else {
						for(Map.Entry<String, SubTree> entry : sMap.entrySet()) {
							String keyword = entry.getKey();
							SubTree st = entry.getValue();
							if(dMap.containsKey(keyword)) {//sMap和dMap取keyword交集
								dDef.add(dMap.get(keyword));
								sDef.add(st);
								commonDef.add(keyword);//共有的keyword放到commonDef
								String absVarName = "var";
								varMap.put(keyword, absVarName);//共有的keyword放到varMap
							}
						}
					}//change pair的DefMap取交集
					for(Map.Entry<String, SubTree> entry : dMap.entrySet()) {
						String keyword = entry.getKey();
						if(!varMap.containsKey(keyword)) {
							String absVarName = "var";
							varMap.put(keyword, absVarName);//共有的keyword放到varMap
						}
					}
					for(Map.Entry<String, SubTree> entry : sMap.entrySet()) {
						String keyword = entry.getKey();
						if(!varMap.containsKey(keyword)) {//sMap和dMap取keyword交集
							String absVarName = "var";
							varMap.put(keyword, absVarName);//共有的keyword放到varMap
						}
					}//私有def分别放入varMap

					String sLine = Output.subtree2src(srcT);
					sLine = Output.absVariable(sLine, varMap);
					if(sLine.equals(srcString)) {
						System.out.println("Find src");
						System.out.println(srcT.getMiName());
						System.out.println(srcT.getRoot().getLine()+","+srcT.getRoot().getColumn());
					}
					String dLine = Output.subtree2src(dstT);
					dLine = Output.absVariable(dLine, varMap);
					if(dLine.equals(tgtString)) {
						System.out.println("Find tgt");
						System.out.println(dstT.getMiName());
						System.out.println(dstT.getRoot().getLine()+","+dstT.getRoot().getColumn());
					}
				}else {
					List<ITree> sLeaves = new ArrayList<>();
					Utils.traverse2Leaf(sRoot, sLeaves);
					List<ITree> dLeaves = new ArrayList<>();
					Utils.traverse2Leaf(dRoot, dLeaves);
					for(ITree leaf : sLeaves) {
						String label = leaf.getLabel();
						if(sDefMap.containsKey(label)) {
							ArrayList<Integer> map = sDefMap.get(label);
							if(map!=null) {
								if(map.size()==1) {
									int defID = map.get(0);
									SubTree st = stMap.get(defID);
									sMap.put(label, st);
								}else {//发现有多个def line同一个关键字的情况，可能发生在不同的method
									for(int id : map) {//只取该条语句的subtree
										if(id==sRoot.getId()) {
											SubTree st = stMap.get(id);
											sMap.put(label, st);
											break;
										}
									}
								}
							}
						}
					}
					for(ITree leaf : dLeaves) {
						String label = leaf.getLabel();
						if(dDefMap.containsKey(label)) {
							ArrayList<Integer> map = dDefMap.get(label);
							if(map!=null) {
								if(map.size()==1) {
									int defID = map.get(0);
									SubTree dt = dtMap.get(defID);
									dMap.put(label, dt);
								}else {//发现有多个def line同一个关键字的情况，可能发生在不同的method
									for(int id : map) {//只取该条语句的subtree
										if(id==dRoot.getId()) {
											SubTree dt = dtMap.get(id);
											dMap.put(label, dt);
											break;
										}
									}
								}
							}
						}
					}
					for(Map.Entry<String, SubTree> entry : sMap.entrySet()) {
						String keyword = entry.getKey();
						if(!varMap.containsKey(keyword)) {//sMap和dMap取keyword交集
							String absVarName = "var";
							varMap.put(keyword, absVarName);//共有的keyword放到varMap
						}
					}
					for(Map.Entry<String, SubTree> entry : dMap.entrySet()) {
						String keyword = entry.getKey();
						if(!varMap.containsKey(keyword)) {
							String absVarName = "var";
							varMap.put(keyword, absVarName);//共有的keyword放到varMap
						}
					}//私有def分别放入varMap

					String sLine = Output.subtree2src(srcT);
					sLine = Output.absVariable(sLine, varMap);
					if(sLine.equals(srcString)) {
						System.out.println("Find src");
						System.out.println(srcT.getMiName());
						System.out.println(srcT.getRoot().getLine()+","+srcT.getRoot().getColumn());
					}
					String dLine = Output.subtree2src(dstT);
					dLine = Output.absVariable(dLine, varMap);
					if(dLine.equals(tgtString)) {
						System.out.println("Find tgt");
						System.out.println(dstT.getMiName());
						System.out.println(dstT.getRoot().getLine()+","+dstT.getRoot().getColumn());
					}
				}
			}
		}
	}

}
