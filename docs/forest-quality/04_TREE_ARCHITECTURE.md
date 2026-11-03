# 4 — Tree architecture

Status: separate morphotype weights integrated; new architecture refinement open.

## Research

[Weber and Penn's original tree model/software](https://www.graphics.stanford.edu/courses/cs348c-95-fall/reader/creation/) is the procedural architecture reference, distinct from instance scaling. [Royal Botanic Gardens Sydney's flora](https://plantnet.rbgsyd.nsw.gov.au/cgi-bin/NSWfl.pl?lvl=sp&name=Dicksonia~antarctica&page=nswfl) describes tree-fern fronds and their differing upper/lower appearance. [RHS morphology](https://www.rhs.org.uk/plants/5794/dicksonia-antarctica/details) identifies an apical frond rosette above an erect rhizome. These references do not make Thessa's fictional forms a taxonomic reconstruction.

## Applicability review

The code already has three actual tree architectures: form 1 has one apical socket; form 0 has a paired fork; form 2 has asymmetric high/low sockets. Previously tree forms were equally weighted; the 50/35/15 weights in adjacent tests refer to ground ferns. Shared woody generators publish crown attachments, so changing stems without updating sockets would detach fronds. Full/reduced/shadow forms must agree.

## Implementation

Keep the existing three anatomical sources and change their authored proportions to 65% single apex, 25% paired fork and 10% asymmetric fork. Stable IDs select forms through the existing hash; density/quality do not change that selection. Source membership and morphotype are authored visual priors, not a botanical growth simulation. A bounded population test checks all three proportions and their one/two attachment sockets. Next add age/lean/asymmetric frond loss through the same woody/attachment generator. Do not randomly warp renderer instances while leaving shadow anatomy unchanged.

## Verification needed

The existing uninterrupted-stem, attachment, outward-normal and full/reduced source tests provide structural checks. Capture all forms at 10 m and in silhouette; scene acceptance still needs independent review. No claimed botanical or visual closure.
