/*
 * This file is part of GumTree.
 *
 * GumTree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GumTree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with GumTree.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2011-2016 Jean-Rémy Falleri <jr.falleri@gmail.com>
 * Copyright 2011-2016 Floréal Morandat <florealm@gmail.com>
 */

package gumtreediff.matchers.heuristic.gt;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.optimal.zs.ZsMatcher;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeMap;

public abstract class AbstractBottomUpMatcher extends Matcher {
    //TODO make final?
    public static int SIZE_THRESHOLD =
            Integer.parseInt(System.getProperty("gt.bum.szt", "100"));
  //seems SIZE_THRESHOLD will significantly affect the speed and scalability of analysis
    public static final double SIM_THRESHOLD =
            Double.parseDouble(System.getProperty("gt.bum.smt", "0.5"));

    protected TreeMap srcIds;
    protected TreeMap dstIds;

    protected TreeMap mappedSrc;
    protected TreeMap mappedDst;

    public AbstractBottomUpMatcher(ITree src, ITree dst, MappingStore store) {
        super(src, dst, store);
        srcIds = new TreeMap(src);
        dstIds = new TreeMap(dst);

        mappedSrc = new TreeMap();
        mappedDst = new TreeMap();
        for (Mapping m : store.asSet()) {
            mappedSrc.putTrees(m.getFirst());
            mappedDst.putTrees(m.getSecond());
        }
    }

    protected List<ITree> getDstCandidates(ITree src) {
        List<ITree> seeds = new ArrayList<>();
        for (ITree c: src.getDescendants()) {
            ITree m = mappings.getDst(c);
            if (m != null) seeds.add(m);
        }
        List<ITree> candidates = new ArrayList<>();
        Set<ITree> visited = new HashSet<>();
        for (ITree seed: seeds) {
            while (seed.getParent() != null) {
                ITree parent = seed.getParent();
                if (visited.contains(parent))
                    break;
                visited.add(parent);
                if (parent.getType() == src.getType() && !isDstMatched(parent) && !parent.isRoot())
                    candidates.add(parent);
                seed = parent;
            }
        }

        return candidates;
    }

    //FIXME checks if it is better or not to remove the already found mappings.
    protected void lastChanceMatch(ITree src, ITree dst) {
        ITree cSrc = src.deepCopy();
        ITree cDst = dst.deepCopy();
        removeMatched(cSrc, true);
        removeMatched(cDst, false);

        HashMap<ITree, ITree> newcandidates = new HashMap<>();
        if (cSrc.getSize() < AbstractBottomUpMatcher.SIZE_THRESHOLD
                || cDst.getSize() < AbstractBottomUpMatcher.SIZE_THRESHOLD) {
            Matcher m = new ZsMatcher(cSrc, cDst, new MappingStore());
            m.match();
            for (Mapping candidate: m.getMappings()) {
                ITree left = srcIds.getTree(candidate.getFirst().getId());
                ITree right = dstIds.getTree(candidate.getSecond().getId());

//                System.out.println("ZsMatcher"+src.getId()+" :"+left.getId()+","+right.getId());
                if (left.getId() == src.getId() || right.getId() == dst.getId()) {
//                    System.err.printf("Trying to map already mapped source node (%d == %d || %d == %d)\n",
//                            left.getId(), src.getId(), right.getId(), dst.getId());
                    continue;
                } else if (!isMappingAllowed(left, right)) {
//                    System.err.printf("Trying to map incompatible nodes (%s, %s)\n",
//                            left.toShortString(), right.toShortString());
                	continue;
                } else {
//                	if(left.getId()==514) {
//                		System.err.println("find514");
//                		Matcher m1 = new ZsMatcher(left, right, new MappingStore());
//                		m1.match();
//                		System.out.println("514size:"+m1.getMappings().asSet().size());
//                		for (Mapping map: m1.getMappings()) {
//                			ITree left1 = map.first;
//                			ITree right1 = map.second;
//                			System.out.println("Recover:"+left1.getId()+","+right1.getId());
//                		}
//                	}
                	double sim = diceSimilarity(left, right);
                	if(!left.isLeaf()){
                		if(sim >= SIM_THRESHOLD) {
                			addMapping(left, right);
//                			System.out.println("ZsMatcher"+src.getId()+" :"+left.getId()+","+right.getId());
                		}else {
                			newcandidates.put(left, right);
                		}
                	}else {
                		addMapping(left, right);
//                		System.out.println("ZsMatcher"+src.getId()+" :"+left.getId()+","+right.getId());
                	}

                }
            }
            for(Map.Entry<ITree, ITree> entry : newcandidates.entrySet()) {
            	ITree left = entry.getKey();
            	ITree right = entry.getValue();
            	double sim = diceSimilarity(left, right);
            	if(sim >= SIM_THRESHOLD) {
        			addMapping(left, right);
        		}
            }
        }
//        mappedSrc.putTrees(src);
//        mappedDst.putTrees(dst);
    }

    /**
     * Remove mapped nodes from the tree. Be careful this method will invalidate
     * all the metrics of this tree and its descendants. If you need them, you need
     * to recompute them.
     */
    public ITree removeMatched(ITree tree, boolean isSrc) {
        for (ITree t: tree.getTrees()) {
            if ((isSrc && isSrcMatched(t)) || ((!isSrc) && isDstMatched(t))) {
                if (t.getParent() != null) t.getParent().getChildren().remove(t);
                t.setParent(null);
            }
        }
        tree.refresh();
        return tree;
    }

    @Override
    public boolean isMappingAllowed(ITree src, ITree dst) {
        return src.hasSameType(dst)
                && !(isSrcMatched(src) || isDstMatched(dst));
    }

    @Override
    protected void addMapping(ITree src, ITree dst) {
        mappedSrc.putTree(src);
        mappedDst.putTree(dst);
        super.addMapping(src, dst);
    }

    boolean isSrcMatched(ITree tree) {
        return mappedSrc.contains(tree);
    }

    boolean isDstMatched(ITree tree) {
        return mappedDst.contains(tree);
    }
}
