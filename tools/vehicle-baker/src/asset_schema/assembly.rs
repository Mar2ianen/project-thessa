//! TOML assembly links and runtime graph construction for procedural bodies.

use super::*;

/// Optional part-assembly section of the vehicle asset.
#[derive(Debug, Default, Deserialize)]
pub(crate) struct AssemblyAsset {
    #[serde(default)]
    pub(crate) links: Vec<AssemblyLinkAsset>,
}

/// One assembly link asset with `body.node` endpoints and its initial hatch state.
#[derive(Debug, Deserialize)]
pub(crate) struct AssemblyLinkAsset {
    pub(crate) name: String,
    pub(crate) parent: String,
    pub(crate) child: String,
    /// Initial hatch state (stack links are always open). Defaults open.
    #[serde(default = "hatch_open_default")]
    pub(crate) hatch_open: bool,
}

fn hatch_open_default() -> bool {
    true
}

/// Resolve authored `body.node` endpoints into validated compiler links.
pub(crate) fn resolve_assembly_links(
    links: &[AssemblyLinkAsset],
) -> Result<Vec<AssemblyLink>, String> {
    fn endpoint(value: &str, link: &str) -> Result<(String, String), String> {
        value.split_once('.').map_or_else(
            || {
                Err(format!(
                    "assembly link '{link}' endpoint '{value}' must be 'body.node'"
                ))
            },
            |(body, node)| Ok((body.into(), node.into())),
        )
    }
    links
        .iter()
        .map(|link| {
            let (parent_body, parent_node) = endpoint(&link.parent, &link.name)?;
            let (child_body, child_node) = endpoint(&link.child, &link.name)?;
            Ok(AssemblyLink {
                name: link.name.clone(),
                parent_body,
                parent_node,
                child_body,
                child_node,
                hatch_open: link.hatch_open,
            })
        })
        .collect()
}

pub(crate) fn runtime_assembly(
    bodies: &[ProceduralBody],
    links: &[AssemblyLinkAsset],
    root_name: &str,
    volumes: Vec<AssemblyVolume>,
) -> Result<VehicleAssembly, String> {
    let body_indices: std::collections::HashMap<&str, usize> = bodies
        .iter()
        .enumerate()
        .map(|(index, body)| (body.name.as_str(), index))
        .collect();
    let root_body = *body_indices
        .get(root_name)
        .ok_or_else(|| format!("assembly root '{root_name}' is missing"))?;
    let mut runtime_links = Vec::with_capacity(links.len());
    for link in links {
        let (parent_name, parent_node_name) = link
            .parent
            .split_once('.')
            .ok_or_else(|| format!("assembly link '{}' has malformed parent", link.name))?;
        let (child_name, child_node_name) = link
            .child
            .split_once('.')
            .ok_or_else(|| format!("assembly link '{}' has malformed child", link.name))?;
        let parent = *body_indices
            .get(parent_name)
            .ok_or_else(|| format!("assembly link '{}' has unknown parent body", link.name))?;
        let child = *body_indices
            .get(child_name)
            .ok_or_else(|| format!("assembly link '{}' has unknown child body", link.name))?;
        let parent_node = bodies[parent]
            .attach_nodes
            .iter()
            .find(|node| node.name == parent_node_name)
            .ok_or_else(|| format!("assembly link '{}' has unknown parent node", link.name))?;
        let child_node = bodies[child]
            .attach_nodes
            .iter()
            .find(|node| node.name == child_node_name)
            .ok_or_else(|| format!("assembly link '{}' has unknown child node", link.name))?;
        let hatch = parent_node.kind == AttachKind::Hatch || child_node.kind == AttachKind::Hatch;
        runtime_links.push(NamedAssemblyLink {
            name: link.name.clone(),
            state: AssemblyLinkState {
                a: parent,
                b: child,
                hatch,
                open: !hatch || link.hatch_open,
            },
        });
    }
    let mut tanks = Vec::new();
    let mut engine_ports = Vec::new();
    for (body_index, body) in bodies.iter().enumerate() {
        for region in &body.regions {
            match region.kind {
                RegionKind::Tank { .. } | RegionKind::FluidTank { .. } => {
                    tanks.push(AssemblyEndpoint {
                        name: format!("{}.{}", body.name, region.name),
                        body: body_index,
                    });
                }
                RegionKind::Bipropellant { .. } => {
                    for component in ["ox", "fuel"] {
                        tanks.push(AssemblyEndpoint {
                            name: format!("{}.{}-{component}", body.name, region.name),
                            body: body_index,
                        });
                    }
                }
                _ => {}
            }
        }
        for port in &body.ports {
            if port.kind == PortKind::EngineMount {
                engine_ports.push(AssemblyEndpoint {
                    name: format!("{}.{}", body.name, port.name),
                    body: body_index,
                });
            }
        }
    }
    let assembly = VehicleAssembly {
        root_body,
        body_names: bodies.iter().map(|body| body.name.clone()).collect(),
        links: runtime_links,
        volumes,
        tanks,
        engine_ports,
    };
    assembly.validate().map_err(|error| error.to_string())?;
    Ok(assembly)
}
