//! Contact state, event summaries, and debug snapshots.

use super::super::*;

impl CollisionWorld {
    /// Number of narrow-phase contact pairs, touching or not.
    pub fn active_contact_pair_count(&self) -> usize {
        self.narrow_phase.contact_pairs().count()
    }

    /// Number of contact pairs with at least one active contact point.
    pub fn touching_contact_pair_count(&self) -> usize {
        self.narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact())
            .count()
    }

    /// Whether a dynamic body currently sleeps. Landed vehicles under steady
    /// load should sleep; a body that never sleeps under constant wrench
    /// points at bad geometry, friction, or solver parameters rather than at
    /// a need for artificial damping.
    pub fn dynamic_body_sleeping(
        &self,
        id: CollisionBodyId,
    ) -> Result<bool, CollisionBackendError> {
        let entry = self
            .dynamic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownBody(id))?;
        self.bodies
            .get(entry.rapier)
            .map(|body| body.is_sleeping())
            .ok_or(CollisionBackendError::BackendStateLost(id))
    }

    /// Serializable telemetry snapshot for debug tooling, regression
    /// fixtures, and the contact MVP probe. Contains no Rapier types.
    pub fn debug_snapshot(&self) -> Result<CollisionDebugSnapshot, CollisionBackendError> {
        let mut dynamic_bodies = Vec::with_capacity(self.dynamic.len());
        for (id, entry) in &self.dynamic {
            let body = self
                .bodies
                .get(entry.rapier)
                .ok_or(CollisionBackendError::BackendStateLost(*id))?;
            let state = self.body_state(*id)?;
            dynamic_bodies.push(CollisionBodyDebug {
                id_raw: id.raw(),
                kinematic: false,
                position_inertial_m: state.position_inertial_m.to_array(),
                velocity_inertial_mps: state.velocity_inertial_mps.to_array(),
                sleeping: body.is_sleeping(),
            });
        }
        let mut kinematic_bodies = Vec::with_capacity(self.kinematic.len());
        for (id, entry) in &self.kinematic {
            let body = self
                .bodies
                .get(entry.rapier)
                .ok_or(CollisionBackendError::BackendStateLostKinematic(*id))?;
            let state = self.kinematic_body_state(*id)?;
            kinematic_bodies.push(CollisionBodyDebug {
                id_raw: id.raw(),
                kinematic: true,
                position_inertial_m: state.position_inertial_m.to_array(),
                velocity_inertial_mps: state.velocity_inertial_mps.to_array(),
                sleeping: body.is_sleeping(),
            });
        }
        Ok(CollisionDebugSnapshot {
            dynamic_bodies,
            kinematic_bodies,
            fixed_collider_count: self.fixed.len(),
            patches: self.patch_debug()?,
            contacts: self.contact_summaries(),
            active_contact_pairs: self.active_contact_pair_count(),
            touching_contact_pairs: self.touching_contact_pair_count(),
        })
    }

    /// Attribute a narrow-phase collider to its stable Thessa party.
    fn party_of(&self, handle: ColliderHandle) -> Option<ContactParty> {
        for (id, entry) in &self.dynamic {
            if entry.colliders.contains(&handle) {
                return Some(ContactParty {
                    kind: ContactPartyKind::Dynamic,
                    id_raw: id.raw(),
                });
            }
        }
        for (id, entry) in &self.kinematic {
            if entry.collider == handle {
                return Some(ContactParty {
                    kind: ContactPartyKind::KinematicTerrain,
                    id_raw: id.raw(),
                });
            }
        }
        for (id, entry) in &self.fixed {
            if entry.handle == handle {
                return Some(ContactParty {
                    kind: ContactPartyKind::StaticTerrain,
                    id_raw: id.raw(),
                });
            }
        }
        None
    }

    fn party_velocity_inertial_mps(&self, party: ContactParty) -> DVec3 {
        match party.kind {
            ContactPartyKind::Dynamic => self
                .dynamic
                .iter()
                .find(|(id, _)| id.raw() == party.id_raw)
                .and_then(|(id, _)| self.body_state(*id).ok())
                .map(|state| state.velocity_inertial_mps)
                .unwrap_or(DVec3::ZERO),
            ContactPartyKind::KinematicTerrain => self
                .kinematic
                .iter()
                .find(|(id, _)| id.raw() == party.id_raw)
                .and_then(|(id, _)| self.kinematic_body_state(*id).ok())
                .map(|state| state.velocity_inertial_mps)
                .unwrap_or(DVec3::ZERO),
            ContactPartyKind::StaticTerrain => DVec3::ZERO,
        }
    }

    /// Reduce every touching pair to physical load evidence, capped at
    /// [`MAX_CONTACT_SUMMARIES`]. Contact points are deliberately omitted:
    /// the pair-local point frame is solver-internal, while normal,
    /// penetration, and approach speed are exact in the inertial frame.
    pub fn contact_summaries(&self) -> Vec<ContactSummary> {
        let mut summaries = Vec::new();
        for pair in self.narrow_phase.contact_pairs() {
            if summaries.len() >= MAX_CONTACT_SUMMARIES {
                break;
            }
            if !pair.has_any_active_contact() {
                continue;
            }
            let Some((manifold, contact)) = pair.find_deepest_contact() else {
                continue;
            };
            let (Some(a), Some(b)) = (self.party_of(pair.collider1), self.party_of(pair.collider2))
            else {
                continue;
            };
            let normal_inertial = self
                .frame
                .vector_to_inertial(from_rapier_vector(manifold.data.normal));
            let velocity_a = self.party_velocity_inertial_mps(a);
            let velocity_b = self.party_velocity_inertial_mps(b);
            summaries.push(ContactSummary {
                a,
                b,
                normal_inertial: normal_inertial.to_array(),
                penetration_m: (-contact.dist).max(0.0),
                approach_speed_mps: (velocity_a - velocity_b).dot(normal_inertial),
            });
        }
        summaries
    }

    /// Take the contact summaries recorded at the last step. A damage or
    /// telemetry consumer drains once per tick; undrained summaries are
    /// replaced, never accumulated.
    pub fn drain_contact_events(&mut self) -> Vec<ContactSummary> {
        std::mem::take(&mut self.last_contacts)
    }

    fn patch_debug(&self) -> Result<Vec<PatchDebug>, CollisionBackendError> {
        let mut patches = Vec::with_capacity(self.fixed.len() + self.kinematic.len());
        for (id, entry) in &self.fixed {
            let orientation = entry.orientation_local;
            patches.push(PatchDebug {
                party: ContactParty {
                    kind: ContactPartyKind::StaticTerrain,
                    id_raw: id.raw(),
                },
                center_inertial_m: self
                    .frame
                    .position_to_inertial(entry.center_local_m)
                    .to_array(),
                half_extents_m: entry.half_extents_m.to_array(),
                orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
            });
        }
        for (id, entry) in &self.kinematic {
            let state = self.kinematic_body_state(*id)?;
            let center_inertial_m = state.position_inertial_m
                + state.orientation_body_to_inertial * entry.shape_offset_m;
            let orientation = state.orientation_body_to_inertial;
            patches.push(PatchDebug {
                party: ContactParty {
                    kind: ContactPartyKind::KinematicTerrain,
                    id_raw: id.raw(),
                },
                center_inertial_m: center_inertial_m.to_array(),
                half_extents_m: entry.half_extents_m.to_array(),
                orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
            });
        }
        Ok(patches)
    }
}
