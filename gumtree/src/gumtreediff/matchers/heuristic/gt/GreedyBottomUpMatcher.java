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
 * Copyright 2011-2015 Jean-R茅my Falleri <jr.falleri@gmail.com>
 * Copyright 2011-2015 Flor茅al Morandat <florealm@gmail.com>
 */

package gumtreediff.matchers.heuristic.gt;

import java.util.List;

import gumtreediff.matchers.MappingStore;
import gumtreediff.tree.ITree;

/**
 * Match the nodes using a bottom-up approach. It browse the nodes of the source and destination trees
 * using a post-order traversal, testing if the two selected trees might be mapped. The two trees are mapped
 * if they are mappable and have a dice coefficient greater than SIM_THRESHOLD. Whenever two trees are mapped
 * a exact ZS algorithm is applied to look to possibly forgotten nodes.
 */
public class GreedyBottomUpMatcher extends AbstractBottomUpMatcher {

    public GreedyBottomUpMatcher(ITree src, ITree dst, MappingStore store) {
        super(src, dst, store);
        System.out.println("GreedyBottomUpMatcher");
    }

    @Override
    public void match() {
        for (ITree t: src.postOrder())  {
            if (t.isRoot()) {
                addMapping(t, this.dst);
                lastChanceMatch(t, this.dst);
                break;
            } else if (!(isSrcMatched(t) || t.isLeaf())) {
                List<ITree> candidates = getDstCandidates(t);
                ITree best = null;
                double max = -1D;

//                if(t.getId()==38) {
//                	System.out.println("candiNum:"+candidates.size());
//                    for (ITree cand: candidates) {
//                        double sim = diceSimilarity(t, cand);
//                        System.out.println("candiID:"+cand.getId()+","+sim);
//                    }
//                }

                for (ITree cand: candidates) {
                    double sim = diceSimilarity(t, cand);
                    if (sim > max && sim >= SIM_THRESHOLD) {
                        max = sim;
                        best = cand;
                    }//如果sim=max，是否应有多个候选集
                }

                if (best != null) {
                    lastChanceMatch(t, best);
                    addMapping(t, best);
//                    System.out.println("AddMatcher:"+t.getId()+","+best.getId());
                }

            }
        }
        //RecoveryMatcher
        for (ITree t: src.postOrder())  {
            if (t.isRoot()) {
                break;
            } else if (!(isSrcMatched(t) || t.isLeaf())) {
                List<ITree> candidates = getDstCandidates(t);
                ITree best = null;
                double max = -1D;


                for (ITree cand: candidates) {
                    double sim = diceSimilarity(t, cand);
                    if (sim > max && sim >= SIM_THRESHOLD) {
                        max = sim;
                        best = cand;
                    }
                }

                if (best != null) {
                    addMapping(t, best);
//                    System.out.println("RecoveryMatcher:"+t.getId()+","+best.getId());
                }

            }
        }
    }
}
