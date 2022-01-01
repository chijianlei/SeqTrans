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
 * Copyright 2011-2015 Jean-Rémy Falleri <jr.falleri@gmail.com>
 * Copyright 2011-2015 Floréal Morandat <florealm@gmail.com>
 */

package gumtreediff.matchers.heuristic.cd;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.simmetrics.StringMetrics;

import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeUtils;

public class ChangeDistillerLeavesMatcher extends Matcher {

    public static final double LABEL_SIM_THRESHOLD = Double.parseDouble(System.getProperty("gt.cd.lsim", "0.5"));

    public ChangeDistillerLeavesMatcher(ITree src, ITree dst, MappingStore store) {
        super(src, dst, store);
    }

    @Override
    public void match() {
        List<Mapping> leavesMappings = new ArrayList<>();
        List<ITree> dstLeaves = retainLeaves(TreeUtils.postOrder(dst));
        for (Iterator<ITree> srcLeaves = TreeUtils.leafIterator(
                TreeUtils.postOrderIterator(src)); srcLeaves.hasNext();) {
            ITree srcLeaf = srcLeaves.next();
            for (ITree dstLeaf: dstLeaves) {
                if (isMappingAllowed(srcLeaf, dstLeaf)) {
                    double sim = StringMetrics.qGramsDistance().compare(srcLeaf.getLabel(), dstLeaf.getLabel());
                    if (sim > LABEL_SIM_THRESHOLD)
                        leavesMappings.add(new Mapping(srcLeaf, dstLeaf));
                }
            }
        }

        Set<ITree> ignoredSrcTrees = new HashSet<>();
        Set<ITree> ignoredDstTrees = new HashSet<>();
        Collections.sort(leavesMappings, new LeafMappingComparator());
        while (leavesMappings.size() > 0) {
            Mapping bestMapping = leavesMappings.remove(0);
            if (!(ignoredSrcTrees.contains(bestMapping.getFirst())
                    || ignoredDstTrees.contains(bestMapping.getSecond()))) {
                addMapping(bestMapping.getFirst(),bestMapping.getSecond());
                ignoredSrcTrees.add(bestMapping.getFirst());
                ignoredDstTrees.add(bestMapping.getSecond());
            }
        }
    }

    public List<ITree> retainLeaves(List<ITree> trees) {
        Iterator<ITree> treeIterator = trees.iterator();
        while (treeIterator.hasNext()) {
            ITree tree = treeIterator.next();
            if (!tree.isLeaf())
                treeIterator.remove();
        }
        return trees;
    }

    private static class LeafMappingComparator implements Comparator<Mapping> {

        @Override
        public int compare(Mapping m1, Mapping m2) {
            return Double.compare(sim(m1), sim(m2));
        }

        public double sim(Mapping m) {
            return StringMetrics.qGramsDistance().compare(m.getFirst().getLabel(), m.getSecond().getLabel());
        }

    }
}
