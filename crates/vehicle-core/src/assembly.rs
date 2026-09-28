//! Part-assembly connectivity (`docs/details/06`).
//!
//! Body-indexed crew/air groups and fuel reachability for runtime hatch
//! toggles. Hangar-side validation (diameters, tree shape, volume
//! inventory) lives in `thessa-fuselage`; this module recomputes pure
//! connectivity from link states with no geometry, so opening or sealing
//! a hatch updates crew, air, and fuel domains through one code path.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

use crate::{
    CabinPressureState, CrewSuitMode, FeedLine, MOLAR_MASS_AIR_G_MOL, MOLAR_MASS_O2_G_MOL,
    PressurizedCabin,
};

/// Assembly connectivity failure modes.
#[derive(Debug, Clone, PartialEq)]
pub enum AssemblyError {
    InvalidLink(String),
    InvalidCabinState(String),
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLink(message) => write!(formatter, "invalid assembly link: {message}"),
            Self::InvalidCabinState(message) => {
                write!(formatter, "invalid assembly cabin state: {message}")
            }
        }
    }
}

impl Error for AssemblyError {}

/// One runtime assembly link between two body indices. Stack
/// links have no door (`hatch = false`) and stay resource-open; hatch
/// links carry the live open state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssemblyLinkState {
    pub a: usize,
    pub b: usize,
    pub hatch: bool,
    pub open: bool,
}

impl AssemblyLinkState {
    pub fn validate(&self, node_count: usize) -> Result<(), AssemblyError> {
        if self.a >= node_count || self.b >= node_count {
            return Err(AssemblyError::InvalidLink(format!(
                "link endpoint out of range for {node_count} nodes"
            )));
        }
        if self.a == self.b {
            return Err(AssemblyError::InvalidLink(
                "link connects a node to itself".into(),
            ));
        }
        Ok(())
    }

    /// Crew passes only through a hatch interface that is currently open.
    pub fn crew_open(&self) -> bool {
        self.hatch && self.open
    }

    /// Fuel passes through stack links always, hatch links only when open.
    pub fn resource_open(&self) -> bool {
        self.open || !self.hatch
    }
}

/// A named hatch/stack link in the baked vehicle. `state.a` and
/// `state.b` index `VehicleAssembly::body_names`, not cabin volumes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NamedAssemblyLink {
    pub name: String,
    pub state: AssemblyLinkState,
    /// Optional authored liquid-feed pipe across this joint. The structure
    /// and cabin topology are unchanged when the line is closed/unavailable.
    #[serde(default)]
    pub feed_line: Option<FeedLine>,
}

/// Explicit non-structural crossfeed edge between two assembly bodies. It
/// carries resource reachability only: it never joins structure, crew, or air.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NamedAssemblyResourceEdge {
    pub name: String,
    pub a: usize,
    pub b: usize,
    pub open: bool,
    /// Optional pressure-loss segment on this non-structural crossfeed.
    #[serde(default)]
    pub feed_line: Option<FeedLine>,
}

impl NamedAssemblyResourceEdge {
    pub fn validate(&self, node_count: usize) -> Result<(), AssemblyError> {
        if self.name.trim().is_empty()
            || self.a >= node_count
            || self.b >= node_count
            || self.a == self.b
        {
            return Err(AssemblyError::InvalidLink(format!(
                "invalid non-structural resource edge '{}'",
                self.name
            )));
        }
        if let Some(line) = self.feed_line {
            line.validate().map_err(|error| {
                AssemblyError::InvalidLink(format!(
                    "invalid feed line on resource edge '{}': {error}",
                    self.name
                ))
            })?;
        }
        Ok(())
    }
}

/// Interior region address in one assembled part.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssemblyVolume {
    pub name: String,
    pub body: usize,
    /// Whether this region has a compiled pressure/air inventory.
    pub pressurized: bool,
    pub volume_m3: f64,
    pub centroid_body_m: DVec3,
    pub seats: u32,
    pub seat_positions_body_m: Vec<DVec3>,
}

/// Resource endpoint attached to a part; tank names and engine-port names
/// use the same qualified names as the hangar compiler.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssemblyEndpoint {
    pub name: String,
    pub body: usize,
}

/// Runtime assembly connectivity retained by `VehicleDefinition`.
/// Physical geometry has already been transformed and merged by the baker;
/// this graph keeps hatch state and compartment/resource reachability live.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleAssembly {
    pub root_body: usize,
    pub body_names: Vec<String>,
    pub links: Vec<NamedAssemblyLink>,
    /// Runtime crossfeed edges outside the structural part tree.
    #[serde(default)]
    pub resource_edges: Vec<NamedAssemblyResourceEdge>,
    pub volumes: Vec<AssemblyVolume>,
    pub tanks: Vec<AssemblyEndpoint>,
    pub engine_ports: Vec<AssemblyEndpoint>,
}

impl VehicleAssembly {
    pub fn validate(&self) -> Result<(), AssemblyError> {
        if self.body_names.is_empty() || self.root_body >= self.body_names.len() {
            return Err(AssemblyError::InvalidLink(
                "assembly root must identify one of its bodies".into(),
            ));
        }
        let mut names = std::collections::HashSet::new();
        if self
            .body_names
            .iter()
            .any(|name| name.trim().is_empty() || !names.insert(name.as_str()))
        {
            return Err(AssemblyError::InvalidLink(
                "body names must be non-empty and unique".into(),
            ));
        }
        if self.links.len() + 1 != self.body_names.len() {
            return Err(AssemblyError::InvalidLink(format!(
                "a {}-body assembly needs {} links (got {})",
                self.body_names.len(),
                self.body_names.len().saturating_sub(1),
                self.links.len()
            )));
        }
        let mut link_names = std::collections::HashSet::new();
        let mut resource_edge_names = std::collections::HashSet::new();
        let mut union = UnionFind::new(self.body_names.len());
        let mut indegree = vec![0_u8; self.body_names.len()];
        for link in &self.links {
            if link.name.trim().is_empty() || !link_names.insert(link.name.as_str()) {
                return Err(AssemblyError::InvalidLink(
                    "link names must be non-empty and unique".into(),
                ));
            }
            link.state.validate(self.body_names.len())?;
            if let Some(line) = link.feed_line {
                line.validate().map_err(|error| {
                    AssemblyError::InvalidLink(format!(
                        "invalid feed line on assembly link '{}': {error}",
                        link.name
                    ))
                })?;
            }
            if link.state.b >= indegree.len() || indegree[link.state.b] != 0 {
                return Err(AssemblyError::InvalidLink(format!(
                    "body '{}' has multiple assembly parents",
                    self.body_names[link.state.b]
                )));
            }
            indegree[link.state.b] = 1;
            if !union.union(link.state.a, link.state.b) {
                return Err(AssemblyError::InvalidLink(format!(
                    "link '{}' closes an assembly cycle",
                    link.name
                )));
            }
            if !link.state.hatch && !link.state.open {
                return Err(AssemblyError::InvalidLink(format!(
                    "stack link '{}' must remain open",
                    link.name
                )));
            }
        }
        for edge in &self.resource_edges {
            edge.validate(self.body_names.len())?;
            if !resource_edge_names.insert(edge.name.as_str()) {
                return Err(AssemblyError::InvalidLink(format!(
                    "duplicate resource edge name '{}'",
                    edge.name
                )));
            }
        }
        if indegree[self.root_body] != 0
            || indegree
                .iter()
                .enumerate()
                .any(|(body, degree)| body != self.root_body && *degree != 1)
        {
            return Err(AssemblyError::InvalidLink(
                "links must form a rooted parent-child tree".into(),
            ));
        }
        let root = union.find(self.root_body);
        if !(0..self.body_names.len()).all(|body| union.find(body) == root) {
            return Err(AssemblyError::InvalidLink(
                "assembly graph is disconnected".into(),
            ));
        }
        let mut volume_names = std::collections::HashSet::new();
        for volume in &self.volumes {
            if volume.body >= self.body_names.len()
                || volume.name.trim().is_empty()
                || !volume_names.insert(volume.name.as_str())
                || !volume.volume_m3.is_finite()
                || volume.volume_m3 <= 0.0
                || !volume.centroid_body_m.is_finite()
                || volume.seats as usize != volume.seat_positions_body_m.len()
                || volume
                    .seat_positions_body_m
                    .iter()
                    .any(|position| !position.is_finite())
            {
                return Err(AssemblyError::InvalidLink(format!(
                    "invalid or duplicate interior volume '{}'",
                    volume.name
                )));
            }
        }
        for endpoint in self.tanks.iter().chain(&self.engine_ports) {
            if endpoint.body >= self.body_names.len() || endpoint.name.trim().is_empty() {
                return Err(AssemblyError::InvalidLink(format!(
                    "invalid assembly resource endpoint '{}'",
                    endpoint.name
                )));
            }
        }
        Ok(())
    }

    /// Open or seal one hatch by its authored link name. Structural stack
    /// joints have no door and cannot be toggled closed here.
    pub fn set_hatch_open(&mut self, name: &str, open: bool) -> Result<(), AssemblyError> {
        let link = self
            .links
            .iter_mut()
            .find(|link| link.name == name)
            .ok_or_else(|| AssemblyError::InvalidLink(format!("unknown link '{name}'")))?;
        if !link.state.hatch {
            return Err(AssemblyError::InvalidLink(format!(
                "link '{name}' is a structural stack joint, not a hatch"
            )));
        }
        link.state.open = open;
        Ok(())
    }

    /// Open or close a named crossfeed edge without changing structural,
    /// crew, or cabin topology.
    pub fn set_resource_edge_open(&mut self, name: &str, open: bool) -> Result<(), AssemblyError> {
        let edge = self
            .resource_edges
            .iter_mut()
            .find(|edge| edge.name == name)
            .ok_or_else(|| AssemblyError::InvalidLink(format!("unknown resource edge '{name}'")))?;
        edge.open = open;
        Ok(())
    }

    /// Partition structural bodies if one named assembly joint fails.
    /// Non-structural resource edges are deliberately ignored: they may keep
    /// fluid reachability but never keep two structural clusters together.
    /// The result is ordered by each component's first body index and is a
    /// topology plan for the caller that rebuilds rigid-body state.
    pub fn body_components_after_link_failure(
        &self,
        failed_link_name: &str,
    ) -> Result<Vec<Vec<usize>>, AssemblyError> {
        self.validate()?;
        if !self.links.iter().any(|link| link.name == failed_link_name) {
            return Err(AssemblyError::InvalidLink(format!(
                "unknown structural link '{failed_link_name}'"
            )));
        }
        let mut adjacency = vec![Vec::new(); self.body_names.len()];
        for link in &self.links {
            if link.name == failed_link_name {
                continue;
            }
            adjacency[link.state.a].push(link.state.b);
            adjacency[link.state.b].push(link.state.a);
        }
        let mut seen = vec![false; self.body_names.len()];
        let mut components = Vec::new();
        for root in 0..self.body_names.len() {
            if seen[root] {
                continue;
            }
            seen[root] = true;
            let mut stack = vec![root];
            let mut component = Vec::new();
            while let Some(body) = stack.pop() {
                component.push(body);
                for neighbor in &adjacency[body] {
                    if !seen[*neighbor] {
                        seen[*neighbor] = true;
                        stack.push(*neighbor);
                    }
                }
            }
            component.sort_unstable();
            components.push(component);
        }
        Ok(components)
    }

    /// Remove one structural connection and return the resulting connected
    /// assembly topologies. A non-structural edge is retained only when both
    /// endpoints remain in the same cluster; a resource umbilical cannot
    /// merge separated rigid bodies. Every returned topology uses local body
    /// indices and validates as an independent assembly.
    pub fn split_after_link_failure(
        &self,
        failed_link_name: &str,
    ) -> Result<Vec<Self>, AssemblyError> {
        let components = self.body_components_after_link_failure(failed_link_name)?;
        let mut split = Vec::with_capacity(components.len());
        for component in components {
            let mut remap = vec![None; self.body_names.len()];
            let body_names = component
                .iter()
                .enumerate()
                .map(|(local, original)| {
                    remap[*original] = Some(local);
                    self.body_names[*original].clone()
                })
                .collect::<Vec<_>>();
            let links = self
                .links
                .iter()
                .filter_map(|link| {
                    if link.name == failed_link_name {
                        return None;
                    }
                    let a = remap[link.state.a]?;
                    let b = remap[link.state.b]?;
                    Some(NamedAssemblyLink {
                        name: link.name.clone(),
                        state: AssemblyLinkState { a, b, ..link.state },
                        feed_line: link.feed_line,
                    })
                })
                .collect();
            let resource_edges = self
                .resource_edges
                .iter()
                .filter_map(|edge| {
                    let a = remap[edge.a]?;
                    let b = remap[edge.b]?;
                    Some(NamedAssemblyResourceEdge {
                        name: edge.name.clone(),
                        a,
                        b,
                        open: edge.open,
                        feed_line: edge.feed_line,
                    })
                })
                .collect();
            let mut assembly = Self {
                root_body: remap[self.root_body].unwrap_or(0),
                body_names,
                links,
                resource_edges,
                volumes: self
                    .volumes
                    .iter()
                    .filter_map(|volume| {
                        let body = remap[volume.body]?;
                        Some(AssemblyVolume {
                            name: volume.name.clone(),
                            body,
                            pressurized: volume.pressurized,
                            volume_m3: volume.volume_m3,
                            centroid_body_m: volume.centroid_body_m,
                            seats: volume.seats,
                            seat_positions_body_m: volume.seat_positions_body_m.clone(),
                        })
                    })
                    .collect(),
                tanks: self
                    .tanks
                    .iter()
                    .filter_map(|endpoint| {
                        Some(AssemblyEndpoint {
                            name: endpoint.name.clone(),
                            body: remap[endpoint.body]?,
                        })
                    })
                    .collect(),
                engine_ports: self
                    .engine_ports
                    .iter()
                    .filter_map(|endpoint| {
                        Some(AssemblyEndpoint {
                            name: endpoint.name.clone(),
                            body: remap[endpoint.body]?,
                        })
                    })
                    .collect(),
            };
            // The root remains the authored root when present; otherwise the
            // first body of this independent cluster becomes its local root.
            if !component.contains(&self.root_body) {
                assembly.root_body = 0;
            }
            assembly.validate()?;
            split.push(assembly);
        }
        Ok(split)
    }

    /// Equalize each connected pressure domain as a single ideal-gas state
    /// transition. Total air, oxygen, and mass-weighted temperature are
    /// conserved; inventories are apportioned by chamber volume.
    pub fn equalize_cabin_states(
        &self,
        cabins: &mut [PressurizedCabin],
    ) -> Result<(), AssemblyError> {
        let air_domains = self.air_groups()?;
        let mut cabin_by_name = std::collections::HashMap::new();
        for (index, cabin) in cabins.iter().enumerate() {
            cabin.validate().map_err(|error| {
                AssemblyError::InvalidCabinState(format!("{}: {error}", cabin.name))
            })?;
            if cabin_by_name.insert(cabin.name.as_str(), index).is_some() {
                return Err(AssemblyError::InvalidCabinState(format!(
                    "duplicate runtime cabin '{}'",
                    cabin.name
                )));
            }
        }
        let mut volume_cabins = Vec::with_capacity(self.volumes.len());
        for volume in &self.volumes {
            if !volume.pressurized {
                volume_cabins.push(None);
                continue;
            }
            let Some(&cabin_index) = cabin_by_name.get(volume.name.as_str()) else {
                return Err(AssemblyError::InvalidCabinState(format!(
                    "pressurized assembly volume '{}' has no runtime cabin",
                    volume.name
                )));
            };
            let cabin = &cabins[cabin_index];
            if (volume.volume_m3 - cabin.volume_m3).abs()
                > 1.0e-9 * volume.volume_m3.max(cabin.volume_m3)
            {
                return Err(AssemblyError::InvalidCabinState(format!(
                    "assembly volume '{}' does not match its runtime cabin volume",
                    volume.name
                )));
            }
            volume_cabins.push(Some(cabin_index));
        }
        for cabin in cabins.iter() {
            if !self
                .volumes
                .iter()
                .any(|volume| volume.pressurized && volume.name == cabin.name)
            {
                return Err(AssemblyError::InvalidCabinState(format!(
                    "runtime cabin '{}' is absent from the assembly volume inventory",
                    cabin.name
                )));
            }
        }

        let mut updated_cabins = cabins.to_vec();
        for domain in air_domains {
            let members: Vec<(usize, usize)> = domain
                .into_iter()
                .filter_map(|volume_index| {
                    volume_cabins[volume_index].map(|cabin_index| (volume_index, cabin_index))
                })
                .collect();
            if members.len() < 2 {
                continue;
            }
            let total_volume_m3: f64 = members
                .iter()
                .map(|(volume_index, _)| self.volumes[*volume_index].volume_m3)
                .sum();
            let total_air_kg: f64 = members
                .iter()
                .map(|(_, cabin_index)| cabins[*cabin_index].air_kg)
                .sum();
            if !total_volume_m3.is_finite() || total_volume_m3 <= 0.0 || !total_air_kg.is_finite() {
                return Err(AssemblyError::InvalidCabinState(
                    "air domain has invalid volume or air inventory".into(),
                ));
            }
            if total_air_kg == 0.0 {
                for (_, cabin_index) in members {
                    updated_cabins[cabin_index].air_kg = 0.0;
                    updated_cabins[cabin_index].state = CabinPressureState::Vacuum;
                }
                continue;
            }

            let total_o2_kg: f64 = members
                .iter()
                .map(|(_, cabin_index)| cabins[*cabin_index].o2_kg())
                .sum();
            let mass_temperature_sum: f64 = members
                .iter()
                .map(|(_, cabin_index)| cabins[*cabin_index].air_kg * cabins[*cabin_index].temp_k)
                .sum();
            let common_temp_k = mass_temperature_sum / total_air_kg;
            let common_o2_fraction =
                (total_o2_kg / total_air_kg) * (MOLAR_MASS_AIR_G_MOL / MOLAR_MASS_O2_G_MOL);
            if !common_temp_k.is_finite()
                || !common_o2_fraction.is_finite()
                || !(0.0..=1.0).contains(&common_o2_fraction)
            {
                return Err(AssemblyError::InvalidCabinState(
                    "air mixing produced invalid gas properties".into(),
                ));
            }
            for (volume_index, cabin_index) in members {
                let volume_m3 = self.volumes[volume_index].volume_m3;
                let cabin = &mut updated_cabins[cabin_index];
                cabin.air_kg = total_air_kg * volume_m3 / total_volume_m3;
                cabin.temp_k = common_temp_k;
                cabin.o2_fraction = common_o2_fraction;
                cabin.state = CabinPressureState::Pressurized;
                cabin.validate().map_err(|error| {
                    AssemblyError::InvalidCabinState(format!("{}: {error}", cabin.name))
                })?;
            }
        }
        cabins.clone_from_slice(&updated_cabins);
        Ok(())
    }

    /// Resolve the authored initial hatch topology before mass properties are
    /// baked. Any pressurized domain already open to an unpressurized body is
    /// vented, then the remaining connected air domains are equalized.
    pub fn resolve_initial_cabin_states(
        &self,
        cabins: &mut [PressurizedCabin],
    ) -> Result<(), AssemblyError> {
        let exposed_cabins = self.cabins_exposed_to_unpressurized_regions()?;
        let mut updated_cabins = cabins.to_vec();
        for name in exposed_cabins {
            let cabin = updated_cabins
                .iter_mut()
                .find(|cabin| cabin.name == name)
                .ok_or_else(|| {
                    AssemblyError::InvalidCabinState(format!(
                        "exposed assembly cabin '{name}' has no runtime cabin"
                    ))
                })?;
            cabin.vent();
        }
        self.equalize_cabin_states(&mut updated_cabins)?;
        cabins.clone_from_slice(&updated_cabins);
        Ok(())
    }

    /// Names of pressure chambers in air domains currently exposed through an
    /// open hatch to a body with no pressurized volume.
    pub fn cabins_exposed_to_unpressurized_regions(&self) -> Result<Vec<String>, AssemblyError> {
        self.validate()?;
        let air_groups = self.air_groups()?;
        let mut exposed = Vec::new();
        for link in &self.links {
            if !link.state.crew_open() {
                continue;
            }
            let has_pressure_volume = |body| {
                self.volumes
                    .iter()
                    .any(|volume| volume.body == body && volume.pressurized)
            };
            let pressure_a = has_pressure_volume(link.state.a);
            let pressure_b = has_pressure_volume(link.state.b);
            if pressure_a == pressure_b {
                continue;
            }
            let pressure_body = if pressure_a {
                link.state.a
            } else {
                link.state.b
            };
            for group in &air_groups {
                if group.iter().any(|index| {
                    let volume = &self.volumes[*index];
                    volume.body == pressure_body && volume.pressurized
                }) {
                    exposed.extend(
                        group
                            .iter()
                            .filter(|index| self.volumes[**index].pressurized)
                            .map(|index| self.volumes[*index].name.clone()),
                    );
                }
            }
        }
        exposed.sort();
        exposed.dedup();
        Ok(exposed)
    }

    /// Compartment-index groups joined by crew-passable open hatches.
    pub fn crew_groups(&self) -> Result<Vec<Vec<usize>>, AssemblyError> {
        self.validate()?;
        let volume_bodies: Vec<usize> = self.volumes.iter().map(|volume| volume.body).collect();
        let links: Vec<AssemblyLinkState> = self.links.iter().map(|link| link.state).collect();
        crew_groups(&volume_bodies, self.body_names.len(), &links)
    }

    /// Pressurized cabin groups that share a currently open air path.
    pub fn air_groups(&self) -> Result<Vec<Vec<usize>>, AssemblyError> {
        self.validate()?;
        let volume_bodies: Vec<usize> = self.volumes.iter().map(|volume| volume.body).collect();
        let links: Vec<AssemblyLinkState> = self.links.iter().map(|link| link.state).collect();
        let pressurized: Vec<bool> = self
            .volumes
            .iter()
            .map(|volume| volume.pressurized)
            .collect();
        air_groups(&volume_bodies, self.body_names.len(), &links, &pressurized)
    }

    /// Named compartment groups convenient for crew-routing and cabin UI.
    pub fn crew_domains(&self) -> Result<Vec<Vec<String>>, AssemblyError> {
        self.named_domains(self.crew_groups()?)
    }

    /// Whether a crew member can traverse between two compiled interior
    /// regions with the current hatch states.
    pub fn crew_can_pass(&self, from: &str, to: &str) -> Result<bool, AssemblyError> {
        let (from, to) = self.volume_pair(from, to)?;
        Ok(self
            .crew_groups()?
            .iter()
            .any(|group| group.contains(&from) && group.contains(&to)))
    }

    /// Whether one crew member can safely traverse the current open-hatch
    /// route. Unsuited and hose-fed crew need a pressurized atmosphere in
    /// every region on the route; self-contained suits also cover dry and
    /// vacuum regions.
    pub fn crew_can_pass_safely(
        &self,
        from: &str,
        to: &str,
        cabins: &[PressurizedCabin],
        suit: CrewSuitMode,
    ) -> Result<bool, AssemblyError> {
        let (from_index, to_index) = self.volume_pair(from, to)?;
        if !self.crew_can_pass(from, to)? {
            return Ok(false);
        }

        let from_body = self.volumes[from_index].body;
        let to_body = self.volumes[to_index].body;
        let Some(route_bodies) = self.crew_body_route(from_body, to_body)? else {
            return Ok(false);
        };
        if suit == CrewSuitMode::SelfContained {
            return Ok(true);
        }

        for volume in self
            .volumes
            .iter()
            .filter(|volume| route_bodies.contains(&volume.body))
        {
            if !volume.pressurized {
                return Ok(false);
            }
            let cabin = cabins
                .iter()
                .find(|cabin| cabin.name == volume.name)
                .ok_or_else(|| {
                    AssemblyError::InvalidCabinState(format!(
                        "pressurized assembly volume '{}' has no runtime cabin",
                        volume.name
                    ))
                })?;
            cabin.validate().map_err(|error| {
                AssemblyError::InvalidCabinState(format!("{}: {error}", cabin.name))
            })?;
            if cabin.air_kg <= 0.0 {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Named pressure-sharing groups for cabin equalization systems.
    pub fn air_domains(&self) -> Result<Vec<Vec<String>>, AssemblyError> {
        let groups = self.air_groups()?;
        Ok(groups
            .into_iter()
            .map(|group| {
                group
                    .into_iter()
                    .filter(|index| self.volumes[*index].pressurized)
                    .map(|index| self.volumes[index].name.clone())
                    .collect()
            })
            .filter(|group: &Vec<String>| !group.is_empty())
            .collect())
    }

    /// Whether two pressurized regions share an open gas path. This is a
    /// connectivity query; mass/energy equalization is handled by cabin
    /// pressure dynamics.
    pub fn cabins_share_air(&self, a: &str, b: &str) -> Result<bool, AssemblyError> {
        let (a, b) = self.volume_pair(a, b)?;
        if !self.volumes[a].pressurized || !self.volumes[b].pressurized {
            return Ok(false);
        }
        Ok(self
            .air_groups()?
            .iter()
            .any(|group| group.contains(&a) && group.contains(&b)))
    }

    /// All tank/engine-port pairs connected by stack links and open hatches.
    pub fn feed_paths(&self) -> Result<Vec<(String, String)>, AssemblyError> {
        self.validate()?;
        let tank_bodies: Vec<usize> = self.tanks.iter().map(|endpoint| endpoint.body).collect();
        let port_bodies: Vec<usize> = self
            .engine_ports
            .iter()
            .map(|endpoint| endpoint.body)
            .collect();
        let reachable = feed_reachable_with_resource_edges(
            self.body_names.len(),
            &self.links.iter().map(|link| link.state).collect::<Vec<_>>(),
            &self.resource_edges,
            &tank_bodies,
            &port_bodies,
        )?;
        let mut paths = Vec::new();
        for (tank_body, port_body) in reachable {
            for tank in self.tanks.iter().filter(|tank| tank.body == tank_body) {
                for port in self
                    .engine_ports
                    .iter()
                    .filter(|port| port.body == port_body)
                {
                    paths.push((tank.name.clone(), port.name.clone()));
                }
            }
        }
        paths.sort();
        Ok(paths)
    }

    fn named_domains(&self, groups: Vec<Vec<usize>>) -> Result<Vec<Vec<String>>, AssemblyError> {
        self.validate()?;
        Ok(groups
            .into_iter()
            .map(|group| {
                group
                    .into_iter()
                    .map(|index| self.volumes[index].name.clone())
                    .collect()
            })
            .collect())
    }

    fn volume_pair(&self, a: &str, b: &str) -> Result<(usize, usize), AssemblyError> {
        self.validate()?;
        let find = |name: &str| {
            self.volumes
                .iter()
                .position(|volume| volume.name == name)
                .ok_or_else(|| AssemblyError::InvalidLink(format!("unknown volume '{name}'")))
        };
        Ok((find(a)?, find(b)?))
    }

    fn crew_body_route(&self, from: usize, to: usize) -> Result<Option<Vec<usize>>, AssemblyError> {
        self.validate()?;
        let mut adjacency = vec![Vec::new(); self.body_names.len()];
        for link in &self.links {
            if link.state.crew_open() {
                adjacency[link.state.a].push(link.state.b);
                adjacency[link.state.b].push(link.state.a);
            }
        }

        let mut parent = vec![None; self.body_names.len()];
        let mut seen = vec![false; self.body_names.len()];
        let mut queue = std::collections::VecDeque::from([from]);
        seen[from] = true;
        while let Some(body) = queue.pop_front() {
            if body == to {
                break;
            }
            for &neighbor in &adjacency[body] {
                if !seen[neighbor] {
                    seen[neighbor] = true;
                    parent[neighbor] = Some(body);
                    queue.push_back(neighbor);
                }
            }
        }
        if !seen[to] {
            return Ok(None);
        }

        let mut route = vec![to];
        let mut body = to;
        while body != from {
            body = parent[body].ok_or_else(|| {
                AssemblyError::InvalidLink("crew route has no parent body".into())
            })?;
            route.push(body);
        }
        Ok(Some(route))
    }
}

fn validate_all(node_count: usize, links: &[AssemblyLinkState]) -> Result<(), AssemblyError> {
    for link in links {
        link.validate(node_count)?;
    }
    Ok(())
}

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(count: usize) -> Self {
        Self {
            parent: (0..count).collect(),
        }
    }

    fn find(&mut self, mut index: usize) -> usize {
        while self.parent[index] != index {
            self.parent[index] = self.parent[self.parent[index]];
            index = self.parent[index];
        }
        index
    }

    fn union(&mut self, a: usize, b: usize) -> bool {
        let (root_a, root_b) = (self.find(a), self.find(b));
        if root_a == root_b {
            return false;
        }
        self.parent[root_a] = root_b;
        true
    }

    fn groups(&mut self, count: usize) -> Vec<Vec<usize>> {
        let mut groups: std::collections::HashMap<usize, Vec<usize>> =
            std::collections::HashMap::new();
        for index in 0..count {
            groups.entry(self.find(index)).or_default().push(index);
        }
        let mut groups: Vec<Vec<usize>> = groups.into_values().collect();
        for group in &mut groups {
            group.sort_unstable();
        }
        groups.sort_unstable();
        groups
    }
}

/// Crew-passable volume groups over an assembly body graph.
/// `volume_bodies[i]` names the body containing volume `i`; link endpoints
/// are body indices. Volumes within one body share the authored open interior.
pub fn crew_groups(
    volume_bodies: &[usize],
    body_count: usize,
    links: &[AssemblyLinkState],
) -> Result<Vec<Vec<usize>>, AssemblyError> {
    validate_volume_bodies(volume_bodies, body_count)?;
    validate_all(body_count, links)?;
    let mut union = UnionFind::new(volume_bodies.len());
    let mut first_by_body = vec![None; body_count];
    for (index, body) in volume_bodies.iter().enumerate() {
        if let Some(first) = first_by_body[*body] {
            union.union(first, index);
        } else {
            first_by_body[*body] = Some(index);
        }
    }
    for link in links {
        if link.crew_open() {
            union_linked_bodies(&mut union, volume_bodies, link.a, link.b, None);
        }
    }
    Ok(union.groups(volume_bodies.len()))
}

/// Shared-air domains over the assembly body graph. Unpressurized volumes
/// stay out of the pressure union, although crew may still pass through them.
pub fn air_groups(
    volume_bodies: &[usize],
    body_count: usize,
    links: &[AssemblyLinkState],
    pressurized: &[bool],
) -> Result<Vec<Vec<usize>>, AssemblyError> {
    if pressurized.len() != volume_bodies.len() {
        return Err(AssemblyError::InvalidLink(
            "pressurized mask must cover every volume".into(),
        ));
    }
    validate_volume_bodies(volume_bodies, body_count)?;
    validate_all(body_count, links)?;
    let mut union = UnionFind::new(volume_bodies.len());
    let mut first_by_body = vec![None; body_count];
    for (index, body) in volume_bodies.iter().enumerate() {
        if pressurized[index] {
            if let Some(first) = first_by_body[*body] {
                union.union(first, index);
            } else {
                first_by_body[*body] = Some(index);
            }
        }
    }
    for link in links {
        if link.crew_open() {
            union_linked_bodies(&mut union, volume_bodies, link.a, link.b, Some(pressurized));
        }
    }
    Ok(union.groups(volume_bodies.len()))
}

fn validate_volume_bodies(volume_bodies: &[usize], body_count: usize) -> Result<(), AssemblyError> {
    if volume_bodies.iter().any(|body| *body >= body_count) {
        return Err(AssemblyError::InvalidLink(
            "interior volume body index out of range".into(),
        ));
    }
    Ok(())
}

fn union_linked_bodies(
    union: &mut UnionFind,
    volume_bodies: &[usize],
    a: usize,
    b: usize,
    eligible: Option<&[bool]>,
) {
    let mut representative = None;
    for (index, body) in volume_bodies.iter().enumerate() {
        if (*body == a || *body == b) && eligible.is_none_or(|mask| mask[index]) {
            if let Some(first) = representative {
                union.union(first, index);
            } else {
                representative = Some(index);
            }
        }
    }
}

/// Tank-to-port fuel reachability through resource-open links.
/// `tanks`/`ports` hold body indices; returns `(tank, port)` pairs.
pub fn feed_reachable(
    body_count: usize,
    links: &[AssemblyLinkState],
    tanks: &[usize],
    ports: &[usize],
) -> Result<Vec<(usize, usize)>, AssemblyError> {
    feed_reachable_with_resource_edges(body_count, links, &[], tanks, ports)
}

/// Tank-to-port reachability through structural resource-open links and
/// explicit non-structural resource edges.
pub fn feed_reachable_with_resource_edges(
    body_count: usize,
    links: &[AssemblyLinkState],
    resource_edges: &[NamedAssemblyResourceEdge],
    tanks: &[usize],
    ports: &[usize],
) -> Result<Vec<(usize, usize)>, AssemblyError> {
    validate_all(body_count, links)?;
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); body_count];
    for link in links {
        if link.resource_open() {
            adjacency[link.a].push(link.b);
            adjacency[link.b].push(link.a);
        }
    }
    for edge in resource_edges {
        edge.validate(body_count)?;
        if edge.open {
            adjacency[edge.a].push(edge.b);
            adjacency[edge.b].push(edge.a);
        }
    }
    let mut pairs = Vec::new();
    for tank in tanks {
        if *tank >= body_count {
            return Err(AssemblyError::InvalidLink(
                "tank body index out of range".into(),
            ));
        }
        let mut seen = vec![false; body_count];
        let mut stack = vec![*tank];
        seen[*tank] = true;
        while let Some(next) = stack.pop() {
            for neighbor in &adjacency[next] {
                if !seen[*neighbor] {
                    seen[*neighbor] = true;
                    stack.push(*neighbor);
                }
            }
        }
        for port in ports {
            if *port >= body_count {
                return Err(AssemblyError::InvalidLink(
                    "port body index out of range".into(),
                ));
            }
            if seen[*port] {
                pairs.push((*tank, *port));
            }
        }
    }
    pairs.sort_unstable();
    Ok(pairs)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn link(a: usize, b: usize, hatch: bool, open: bool) -> AssemblyLinkState {
        AssemblyLinkState { a, b, hatch, open }
    }

    #[test]
    fn open_hatch_shares_crew_and_air() {
        let links = vec![link(0, 1, true, true)];
        assert_eq!(crew_groups(&[0, 1], 2, &links).unwrap(), vec![vec![0, 1]]);
        assert_eq!(
            air_groups(&[0, 1], 2, &links, &[true, true]).unwrap(),
            vec![vec![0, 1]]
        );
        // Dry volume on one side: crew passes, air does not mix.
        assert_eq!(
            air_groups(&[0, 1], 2, &links, &[true, false]).unwrap(),
            vec![vec![0], vec![1]]
        );
        // Multiple regions in one part are joined through its open interior.
        assert_eq!(
            crew_groups(&[0, 0, 1], 2, &links).unwrap(),
            vec![vec![0, 1, 2]]
        );
    }

    #[test]
    fn sealed_hatch_splits_domains() {
        let links = vec![link(0, 1, true, false)];
        assert_eq!(
            crew_groups(&[0, 1], 2, &links).unwrap(),
            vec![vec![0], vec![1]]
        );
        assert_eq!(
            air_groups(&[0, 1], 2, &links, &[true, true]).unwrap(),
            vec![vec![0], vec![1]]
        );
        // Sealed hatch still blocks fuel (no crossfeed through doors).
        assert!(feed_reachable(2, &links, &[0], &[1]).unwrap().is_empty());
        assert_eq!(feed_reachable(2, &links, &[0], &[0]).unwrap(), vec![(0, 0)]);
    }

    #[test]
    fn stack_links_always_flow() {
        let links = vec![link(0, 1, false, true)];
        assert_eq!(
            crew_groups(&[0, 1], 2, &links).unwrap(),
            vec![vec![0], vec![1]]
        );
        assert_eq!(feed_reachable(2, &links, &[0], &[1]).unwrap(), vec![(0, 1)]);
    }

    #[test]
    fn chain_reaches_through_middle_bodies() {
        let links = vec![link(0, 1, false, true), link(1, 2, true, true)];
        assert_eq!(
            crew_groups(&[0, 1, 2], 3, &links).unwrap(),
            vec![vec![0], vec![1, 2]]
        );
        assert_eq!(feed_reachable(3, &links, &[0], &[2]).unwrap(), vec![(0, 2)]);
    }

    #[test]
    fn failed_structural_link_partitions_bodies_without_using_resource_edges() {
        let assembly = VehicleAssembly {
            root_body: 0,
            body_names: vec!["core".into(), "booster".into(), "fairing".into()],
            links: vec![
                NamedAssemblyLink {
                    name: "core-booster".into(),
                    state: link(0, 1, false, true),
                    feed_line: None,
                },
                NamedAssemblyLink {
                    name: "booster-fairing".into(),
                    state: link(1, 2, false, true),
                    feed_line: None,
                },
            ],
            resource_edges: vec![
                NamedAssemblyResourceEdge {
                    name: "temporary-umbilical".into(),
                    a: 0,
                    b: 2,
                    open: true,
                    feed_line: None,
                },
                NamedAssemblyResourceEdge {
                    name: "booster-service-line".into(),
                    a: 1,
                    b: 2,
                    open: true,
                    feed_line: None,
                },
            ],
            volumes: Vec::new(),
            tanks: Vec::new(),
            engine_ports: Vec::new(),
        };

        assert_eq!(
            assembly
                .body_components_after_link_failure("core-booster")
                .unwrap(),
            vec![vec![0], vec![1, 2]]
        );
        let split = assembly.split_after_link_failure("core-booster").unwrap();
        assert_eq!(split.len(), 2);
        assert_eq!(split[0].body_names, vec!["core"]);
        assert!(split[0].resource_edges.is_empty());
        assert_eq!(split[1].body_names, vec!["booster", "fairing"]);
        assert_eq!(split[1].links.len(), 1);
        assert_eq!(split[1].resource_edges.len(), 1);
        assert_eq!(split[1].resource_edges[0].name, "booster-service-line");
        assert!(split.iter().all(|cluster| cluster.validate().is_ok()));
        assert!(
            assembly
                .body_components_after_link_failure("missing-link")
                .is_err()
        );
    }

    #[test]
    fn bad_links_fail_closed() {
        assert!(crew_groups(&[0, 1], 2, &[link(0, 2, true, true)]).is_err());
        assert!(crew_groups(&[0, 1], 2, &[link(1, 1, true, true)]).is_err());
        assert!(crew_groups(&[0, 2], 2, &[]).is_err());
        assert!(air_groups(&[0, 1], 2, &[], &[true]).is_err());
        assert!(feed_reachable(2, &[link(0, 1, false, true)], &[5], &[1]).is_err());
    }

    #[test]
    fn vehicle_assembly_hatch_toggles_crew_air_and_feed_domains() {
        let mut assembly = VehicleAssembly {
            root_body: 0,
            body_names: vec!["stage".into(), "capsule".into()],
            links: vec![NamedAssemblyLink {
                name: "crew-hatch".into(),
                state: link(0, 1, true, true),
                feed_line: None,
            }],
            resource_edges: Vec::new(),
            volumes: vec![
                AssemblyVolume {
                    name: "stage.service-bay".into(),
                    body: 0,
                    pressurized: true,
                    volume_m3: 4.0,
                    centroid_body_m: DVec3::ZERO,
                    seats: 0,
                    seat_positions_body_m: Vec::new(),
                },
                AssemblyVolume {
                    name: "capsule.cabin".into(),
                    body: 1,
                    pressurized: true,
                    volume_m3: 8.0,
                    centroid_body_m: DVec3::new(4.0, 0.0, 0.0),
                    seats: 2,
                    seat_positions_body_m: vec![
                        DVec3::new(4.0, -0.5, 0.0),
                        DVec3::new(4.0, 0.5, 0.0),
                    ],
                },
            ],
            tanks: vec![AssemblyEndpoint {
                name: "stage.lox".into(),
                body: 0,
            }],
            engine_ports: vec![AssemblyEndpoint {
                name: "capsule.engine".into(),
                body: 1,
            }],
        };
        assert_eq!(
            assembly.crew_domains().unwrap(),
            vec![vec![
                "stage.service-bay".to_string(),
                "capsule.cabin".to_string()
            ]]
        );
        assert!(
            assembly
                .crew_can_pass("stage.service-bay", "capsule.cabin")
                .unwrap()
        );
        assert!(
            assembly
                .cabins_share_air("stage.service-bay", "capsule.cabin")
                .unwrap()
        );
        assert_eq!(
            assembly.air_domains().unwrap(),
            vec![vec![
                "stage.service-bay".to_string(),
                "capsule.cabin".to_string()
            ]]
        );
        assert_eq!(
            assembly.feed_paths().unwrap(),
            vec![("stage.lox".to_string(), "capsule.engine".to_string())]
        );

        assembly.set_hatch_open("crew-hatch", false).unwrap();
        assert_eq!(
            assembly.crew_domains().unwrap(),
            vec![
                vec!["stage.service-bay".to_string()],
                vec!["capsule.cabin".to_string()]
            ]
        );
        assert!(
            !assembly
                .crew_can_pass("stage.service-bay", "capsule.cabin")
                .unwrap()
        );
        assert!(
            !assembly
                .cabins_share_air("stage.service-bay", "capsule.cabin")
                .unwrap()
        );
        assert_eq!(
            assembly.air_domains().unwrap(),
            vec![
                vec!["stage.service-bay".to_string()],
                vec!["capsule.cabin".to_string()]
            ]
        );
        assert!(assembly.feed_paths().unwrap().is_empty());
        assert!(assembly.set_hatch_open("unknown", true).is_err());
    }

    #[test]
    fn initial_open_hatch_to_dry_region_vents_the_connected_pressure_domain() {
        let assembly = VehicleAssembly {
            root_body: 0,
            body_names: vec!["pressurized-part".into(), "dry-part".into()],
            links: vec![NamedAssemblyLink {
                name: "initial-hatch".into(),
                state: link(0, 1, true, true),
                feed_line: None,
            }],
            resource_edges: Vec::new(),
            volumes: vec![
                AssemblyVolume {
                    name: "cabin".into(),
                    body: 0,
                    pressurized: true,
                    volume_m3: 2.0,
                    centroid_body_m: DVec3::ZERO,
                    seats: 0,
                    seat_positions_body_m: Vec::new(),
                },
                AssemblyVolume {
                    name: "service-bay".into(),
                    body: 1,
                    pressurized: false,
                    volume_m3: 1.0,
                    centroid_body_m: DVec3::X,
                    seats: 0,
                    seat_positions_body_m: Vec::new(),
                },
            ],
            tanks: Vec::new(),
            engine_ports: Vec::new(),
        };
        let initial_air_kg = 2.0 * 101_325.0 / (crate::R_DRY_AIR_J_KG_K * 293.15);
        let mut cabins = vec![
            PressurizedCabin::new("cabin", 2.0, 101.325, 293.15, 0.21, initial_air_kg)
                .expect("pressurized cabin"),
        ];

        assert_eq!(
            assembly.cabins_exposed_to_unpressurized_regions().unwrap(),
            vec!["cabin"]
        );
        assembly
            .resolve_initial_cabin_states(&mut cabins)
            .expect("initial open hatch vents the exposed air");

        assert_eq!(cabins[0].air_kg, 0.0);
        assert_eq!(cabins[0].state, CabinPressureState::Vacuum);
    }

    #[test]
    fn vehicle_assembly_rejects_cycles_forests_and_closed_stack_links() {
        let mut assembly = VehicleAssembly {
            root_body: 0,
            body_names: vec!["root".into(), "child".into()],
            links: vec![NamedAssemblyLink {
                name: "rigid-joint".into(),
                state: link(0, 1, false, true),
                feed_line: None,
            }],
            resource_edges: Vec::new(),
            volumes: Vec::new(),
            tanks: Vec::new(),
            engine_ports: Vec::new(),
        };
        assert!(assembly.validate().is_ok());
        assert!(assembly.set_hatch_open("rigid-joint", false).is_err());
        assembly.links[0].state.open = false;
        assert!(assembly.validate().is_err());
        assembly.links[0].state = link(0, 0, true, true);
        assert!(assembly.validate().is_err());
    }
}
