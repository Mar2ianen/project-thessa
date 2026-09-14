//! Baked-gravity source hierarchy and runtime monopole aggregates
//! (docs/23_GRAVITY_FIELD_COHORTS.md sections 3-5, implementation steps 2-4).
//!
//! The astronomical topology is known and stable, so no generic Barnes-Hut
//! tree is rebuilt per tick. Instead [`GravitySourceTree`] inverts the baked
//! parent links once: every body that is a gravity source, or an ancestor of
//! at least two of them, becomes a node. Synthetic barycentres (bodies with
//! `gravity_source == false`) are exactly the aggregate nodes the baker
//! already tracks — they carry no mass of their own, only children.
//!
//! Per-tick node data (barycenter, internal radius bound) resolves from one
//! [`EphemerisFrame`](crate::EphemerisFrame), once per tick, never per
//! target. Per-target traversal then opens nodes or accepts their monopole
//! under a conservative acceleration-error bound derived below. The exact
//! [`GravityField`](crate::GravityField) path is untouched and remains the
//! authoritative fallback.

use glam::DVec3;

use crate::{BakedEphemeris, BodyId, BodyState, GravityError};

/// One hierarchy node: a baked body plus the source subtree beneath it.
#[derive(Debug, Clone)]
pub struct GravityNode {
    /// Baked body this node represents (source, ancestor, or both).
    pub body: BodyId,
    /// Mass of the body itself when it is a gravity source, else 0.
    pub own_mu: f64,
    /// `own_mu` plus every descendant source mu. Static: mus never change.
    pub mu_total: f64,
    /// Child node indices, sorted by [`BodyId`] for deterministic traversal.
    pub children: Vec<u32>,
}

/// Static source hierarchy over a baked ephemeris.
#[derive(Debug, Clone)]
pub struct GravitySourceTree {
    nodes: Vec<GravityNode>,
    roots: Vec<u32>,
}

impl GravitySourceTree {
    /// Invert baked parent links into aggregate nodes. Pure metadata:
    /// building the tree changes no gravity result. Acyclicity is not
    /// assumed here — it is enforced by [`BakedEphemeris`] validation, so
    /// every parent walk below terminates.
    pub fn build(ephemeris: &BakedEphemeris) -> Result<Self, crate::EphemerisError> {
        // Ancestor bodies that group at least two gravity sources, plus every
        // source itself: those are the nodes. A source with no source
        // relatives still gets its own leaf node so traversal is uniform.
        let mut involved = vec![false; ephemeris.bodies.len()];
        for source in ephemeris.gravity_sources() {
            involved[source.id.index()] = true;
            let mut cursor = source.parent;
            while let Some(parent) = cursor {
                let body = ephemeris.body(parent)?;
                involved[parent.index()] = true;
                cursor = body.parent;
            }
        }
        // Drop ancestors that group fewer than two sources and are not
        // sources themselves: they would aggregate a single child.
        let mut descendant_sources = vec![0_u32; ephemeris.bodies.len()];
        for source in ephemeris.gravity_sources() {
            let mut cursor = source.parent;
            while let Some(parent) = cursor {
                descendant_sources[parent.index()] += 1;
                cursor = ephemeris.body(parent)?.parent;
            }
        }
        let mut node_index = vec![u32::MAX; ephemeris.bodies.len()];
        let mut nodes = Vec::new();
        for body in &ephemeris.bodies {
            let is_source = body.gravity_source;
            let groups_several = descendant_sources[body.id.index()] >= 2;
            if !involved[body.id.index()] || (!is_source && !groups_several) {
                continue;
            }
            node_index[body.id.index()] = nodes.len() as u32;
            nodes.push(GravityNode {
                body: body.id,
                own_mu: if is_source { body.mu } else { 0.0 },
                mu_total: if is_source { body.mu } else { 0.0 },
                children: Vec::new(),
            });
        }
        for node_id in 0..nodes.len() {
            let parent = ephemeris.body(nodes[node_id].body)?.parent;
            let mut cursor = parent;
            while let Some(ancestor) = cursor {
                if node_index[ancestor.index()] != u32::MAX {
                    nodes[node_index[ancestor.index()] as usize]
                        .children
                        .push(node_id as u32);
                    break;
                }
                cursor = ephemeris.body(ancestor)?.parent;
            }
        }
        for node in &mut nodes {
            node.children.sort_unstable();
        }
        // Bottom-up mu totals by memoized depth-first accumulation: no
        // ordering assumption on node indices (bodies are not guaranteed
        // parent-first).
        let mut memo = vec![None; nodes.len()];
        for node_id in 0..nodes.len() {
            let total = accumulate_mu(&nodes, node_id, &mut memo);
            nodes[node_id].mu_total = total;
        }
        let mut roots = Vec::new();
        for (node_id, node) in nodes.iter().enumerate() {
            let mut cursor = ephemeris.body(node.body)?.parent;
            let mut has_node_ancestor = false;
            while let Some(ancestor) = cursor {
                if node_index[ancestor.index()] != u32::MAX {
                    has_node_ancestor = true;
                    break;
                }
                cursor = ephemeris.body(ancestor)?.parent;
            }
            if !has_node_ancestor {
                roots.push(node_id as u32);
            }
        }
        roots.sort_unstable();
        Ok(Self { nodes, roots })
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn nodes(&self) -> &[GravityNode] {
        &self.nodes
    }

    pub fn roots(&self) -> &[u32] {
        &self.roots
    }

    /// Resolve per-tick node frames from one ephemeris frame: barycenter and
    /// conservative internal radius per node. O(nodes), once per tick.
    /// Depth-first with memoization: no ordering assumption on nodes.
    pub fn resolve(&self, states: &[BodyState]) -> Result<Vec<GravityNodeFrame>, GravityError> {
        let mut frames: Vec<Option<GravityNodeFrame>> = vec![None; self.nodes.len()];
        for node_id in 0..self.nodes.len() {
            self.resolve_node(states, node_id as u32, &mut frames)?;
        }
        Ok(frames
            .into_iter()
            .map(|frame| frame.expect("every node resolved"))
            .collect())
    }

    fn resolve_node(
        &self,
        states: &[BodyState],
        node_id: u32,
        frames: &mut [Option<GravityNodeFrame>],
    ) -> Result<(), GravityError> {
        if frames[node_id as usize].is_some() {
            return Ok(());
        }
        let children = self.nodes[node_id as usize].children.clone();
        for child in children {
            self.resolve_node(states, child, frames)?;
        }
        let node = &self.nodes[node_id as usize];
        let own_state = states
            .get(node.body.index())
            .ok_or(crate::EphemerisError::UnknownBody(node.body))?;
        // Weighted barycenter over the node's own mass (when it is a
        // source) and the resolved child barycenters.
        let mut weighted = own_state.position_inertial * node.own_mu;
        let mut mass = node.own_mu;
        for child in &node.children {
            let child_frame = frames[*child as usize].as_ref().expect("child resolved");
            let child_mu = self.nodes[*child as usize].mu_total;
            weighted += child_frame.barycenter * child_mu;
            mass += child_mu;
        }
        // A node always covers at least one source, so mass is positive;
        // guard anyway to keep radius finite on degenerate input.
        let barycenter = if mass > 0.0 && mass.is_finite() {
            weighted / mass
        } else {
            own_state.position_inertial
        };
        // Conservative internal radius around the barycenter: every
        // member (own body when massive, child balls) must fit inside.
        // Canonical gravity is point-mass, so a massive body's own support
        // radius is 0: the mass sits at the center, and the physical body
        // radius belongs to collision/surface systems, not the monopole
        // error. (Inside a physical body the point-mass model itself is
        // what it is — identical in the exact path.)
        let mut anchored = if node.own_mu > 0.0 {
            (own_state.position_inertial - barycenter).length()
        } else {
            0.0
        };
        for child in &node.children {
            let child_frame = frames[*child as usize].as_ref().expect("child resolved");
            anchored =
                anchored.max((child_frame.barycenter - barycenter).length() + child_frame.radius_m);
        }
        frames[node_id as usize] = Some(GravityNodeFrame {
            barycenter,
            radius_m: anchored,
        });
        Ok(())
    }

    /// Evaluate gravity at one target through the hierarchy: open nodes whose
    /// monopole error estimate does not fit the *remaining* budget, accept
    /// the rest. The budget is spent as traversal proceeds, so the returned
    /// bound never exceeds the allocated total however many aggregates are
    /// accepted. Traversal order is fixed (ascending roots, ascending
    /// children), hence deterministic and worker-count independent.
    /// Returns the acceleration plus the achieved (summed) error bound, so
    /// callers can check the total against their allocated budget.
    pub fn evaluate(
        &self,
        frames: &[GravityNodeFrame],
        states: &[BodyState],
        position: DVec3,
        budget_mps2: f64,
    ) -> Result<TreeEval, GravityError> {
        if !budget_mps2.is_finite() || budget_mps2 < 0.0 {
            return Err(GravityError::NonFinite {
                body_id: self
                    .nodes
                    .first()
                    .map(|node| node.body)
                    .unwrap_or(BodyId(0)),
            });
        }
        // Frames index by node id: a short slice (or one from another tree)
        // must fail open, never panic inside the authoritative core.
        if frames.len() != self.nodes.len() {
            return Err(GravityError::Ephemeris(crate::EphemerisError::InvalidBody(
                format!(
                    "gravity frame length {} does not match {} tree nodes",
                    frames.len(),
                    self.nodes.len()
                ),
            )));
        }
        let mut total = DVec3::ZERO;
        let mut error_bound = 0.0;
        let mut remaining = budget_mps2;
        let mut nodes_visited = 0_u32;
        let mut terms_exact = 0_u32;
        // Ascending pop order: roots pushed reversed, children extended
        // reversed, so every pop takes the smallest pending node id.
        let mut stack: Vec<u32> = self.roots.iter().rev().copied().collect();
        while let Some(node_id) = stack.pop() {
            let node = &self.nodes[node_id as usize];
            let frame = &frames[node_id as usize];
            nodes_visited += 1;
            if node.children.is_empty() {
                total += exact_term(states, node.body, node.own_mu, position)?;
                terms_exact += 1;
                continue;
            }
            let offset = frame.barycenter - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() {
                return Err(GravityError::NonFinite { body_id: node.body });
            }
            let distance = distance_squared.sqrt();
            if distance <= frame.radius_m {
                // Target inside the aggregate ball: open unconditionally.
                if node.own_mu > 0.0 {
                    total += exact_term(states, node.body, node.own_mu, position)?;
                    terms_exact += 1;
                }
                stack.extend(node.children.iter().rev());
                continue;
            }
            let clearance = distance - frame.radius_m;
            // Conservative monopole error (mean-value bound on y/|y|^3,
            // whose Jacobian has spectral norm 2/|y|^3):
            //   E <= 2 * mu_total * R / (D - R)^3.
            let estimate = 2.0 * node.mu_total * frame.radius_m / clearance.powi(3);
            if estimate <= remaining {
                let inverse = distance_squared.sqrt().recip();
                if !inverse.is_finite() {
                    return Err(GravityError::NonFinite { body_id: node.body });
                }
                total += offset * (node.mu_total * inverse.powi(3));
                error_bound += estimate;
                remaining -= estimate;
            } else {
                if node.own_mu > 0.0 {
                    total += exact_term(states, node.body, node.own_mu, position)?;
                    terms_exact += 1;
                }
                stack.extend(node.children.iter().rev());
            }
        }
        if total.is_finite() {
            Ok(TreeEval {
                acceleration: total,
                error_bound_mps2: error_bound,
                nodes_visited,
                terms_exact,
            })
        } else {
            Err(GravityError::NonFinite {
                body_id: self
                    .nodes
                    .first()
                    .map(|node| node.body)
                    .unwrap_or(BodyId(0)),
            })
        }
    }
}

/// Per-tick aggregate data for one hierarchy node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravityNodeFrame {
    pub barycenter: DVec3,
    pub radius_m: f64,
}

/// Hierarchy evaluation result: acceleration plus the achieved error bound
/// (summed accepted-node estimates, always within the allocated budget)
/// and traversal telemetry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TreeEval {
    pub acceleration: DVec3,
    pub error_bound_mps2: f64,
    pub nodes_visited: u32,
    pub terms_exact: u32,
}

/// Memoized depth-first subtree mass (own mu plus descendants), summed once
/// per edge and without assuming any node index order.
fn accumulate_mu(nodes: &[GravityNode], node_id: usize, memo: &mut [Option<f64>]) -> f64 {
    if let Some(total) = memo[node_id] {
        return total;
    }
    // Mark in-progress against pathological cycles (never produced by the
    // builder, which follows a DAG of parent links).
    memo[node_id] = Some(nodes[node_id].own_mu);
    let mut total = nodes[node_id].own_mu;
    for child in nodes[node_id].children.clone() {
        total += accumulate_mu(nodes, child as usize, memo);
    }
    memo[node_id] = Some(total);
    total
}

/// One exact point-mass term with the same checks as
/// [`GravityField`](crate::GravityField) accumulation.
fn exact_term(
    states: &[BodyState],
    body: BodyId,
    mu: f64,
    position: DVec3,
) -> Result<DVec3, GravityError> {
    let state = states
        .get(body.index())
        .ok_or(crate::EphemerisError::UnknownBody(body))?;
    let offset = state.position_inertial - position;
    let distance_squared = offset.length_squared();
    if !distance_squared.is_finite() {
        return Err(GravityError::NonFinite { body_id: body });
    }
    if distance_squared == 0.0 {
        return Err(GravityError::Singularity { body_id: body });
    }
    let inverse_distance = distance_squared.sqrt().recip();
    Ok(offset * (mu * inverse_distance.powi(3)))
}

/// Monopole error estimate used by the opening criterion, exposed for
/// differential tests: `2 * mu_total * radius / clearance^3`.
pub fn monopole_error_estimate(mu_total: f64, radius_m: f64, clearance_m: f64) -> f64 {
    2.0 * mu_total * radius_m / clearance_m.powi(3)
}
