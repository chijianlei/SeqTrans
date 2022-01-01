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

package gumtreediff.matchers.heuristic.gt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.MultiMappingStore;
import gumtreediff.tree.ITree;

public class GreedySubtreeMatcher extends AbstractSubtreeMatcher {

    public GreedySubtreeMatcher(ITree src, ITree dst, MappingStore store) {
        super(src, dst, store);
//        System.out.println("GreedySubtreeMatcher");
    }

    @Override
    public void filterMappings(MultiMappingStore multiMappings) {
        // Select unique mappings first and extract ambiguous mappings.
        List<Mapping> ambiguousList = new ArrayList<>();
        Set<ITree> ignored = new HashSet<>();
        for (ITree src: multiMappings.getSrcs()) {
        	boolean isMappingUnique = false;
            if (multiMappings.isSrcUnique(src)) {
                if (multiMappings.isSrcUnique(src)) {
                    ITree dst = multiMappings.getDst(src).iterator().next();
                    if (multiMappings.isDstUnique(dst)) {
                        addMappingRecursively(src, dst);
                        isMappingUnique = true;
                    }
                }
            }
            if (!(ignored.contains(src) || isMappingUnique)) {
                Set<ITree> adsts = multiMappings.getDst(src);
                Set<ITree> asrcs = multiMappings.getSrc(multiMappings.getDst(src).iterator().next());
                for (ITree asrc : asrcs)
                    for (ITree adst: adsts)
                        ambiguousList.add(new Mapping(asrc, adst));
                ignored.addAll(asrcs);
            }
        }
        for(Mapping map : ambiguousList) {
        	ITree src = map.first;
        	ITree dst = map.second;
//        	System.out.println("ambiguousMap:"+src.getId()+","+dst.getId());
        }

        // Rank the mappings by score.
        Set<ITree> srcIgnored = new HashSet<>();
        Set<ITree> dstIgnored = new HashSet<>();
        Collections.sort(ambiguousList, new SiblingsMappingComparator(ambiguousList, mappings, getMaxTreeSize()));

        // Select the best ambiguous mappings
        retainBestMapping(ambiguousList, srcIgnored, dstIgnored);
    }

}
