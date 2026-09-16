//! Shared canonical world field, bounded async cube-sphere terrain and local survey.
use super::*;
use bevy::{
    asset::RenderAssetUsages,
    ecs::system::SystemParam,
    math::{DQuat, DVec3},
    mesh::{Indices, PrimitiveTopology},
    tasks::{AsyncComputeTaskPool, Task, block_on, poll_once},
};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
    time::Instant,
};
use thessa_bevy_rcbt::{
    CbtFrameInput, CbtRenderMaterial, CbtRenderPages, CbtRenderState, CbtRenderSurface,
    CbtRenderTopology, RenderView,
};
use thessa_rcbt_core::{
    CandidateAction, HeightPage, LeafCandidate, Node as CbtNode, Tree, WorkClass,
};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TerrainTile, TileKey},
};

#[derive(SystemParam)]
struct CbtTerrain<'w> {
    state: Res<'w, CbtRenderState>,
    input: ResMut<'w, CbtFrameInput>,
    topology: ResMut<'w, CbtRenderTopology>,
    pages: ResMut<'w, CbtRenderPages>,
    material_pages: ResMut<'w, thessa_bevy_rcbt::CbtRenderMaterialPages>,
    surface: ResMut<'w, CbtRenderSurface>,
    material: Option<Res<'w, CbtRenderMaterial>>,
    graphics: Option<Res<'w, GraphicsResolved>>,
}

#[derive(Resource)]
pub(super) struct WorldTerrain {
    pub field: Arc<PlanetField>,
    pub counters: thessa_perf::WorldCounters,
    pub cache_bytes: u64,
    pub render_center: Option<Vec3>,
    pub render_origin_m: DVec3,
    cache: BTreeMap<TileKey, CachedTile>,
    jobs: BTreeMap<TileKey, Task<TerrainBuildOutput>>,
    material_jobs: BTreeMap<TileKey, Task<thessa_bevy_rcbt::CbtMaterialPage>>,
    wanted: Vec<TileKey>,
    visible: BTreeSet<TileKey>,
    selection_at: f64,
    /// A bounded CBT plan may take several PostUpdate frames to converge.
    topology_pending: bool,
    /// Selection-frame eye (body frame) and camera forward driving the
    /// movement trigger and the velocity LOD bias below.
    selected_eye: DVec3,
    selected_forward: DVec3,
    selected_valid: bool,
    /// Smoothed eye speed (m/s) for the velocity detail bias.
    eye_speed_mps: f64,
}
struct CachedTile {
    mesh: Option<Handle<Mesh>>,
    anchor: DVec3,
    vertices: u64,
    triangles: u64,
    texture_bytes: u64,
    material: Option<Handle<StandardMaterial>>,
    images: Option<[Handle<Image>; 3]>,
}

/// Finished worker output: built mesh plus upload-ready images.
/// Mipmap generation (`surface_image`, ~65k sRGB powf per 128 px tile) and
/// mesh assembly (`generate_tangents`) run on the pool, never on the frame
/// thread — per finished tile the main thread only inserts asset handles.
struct TerrainBuildOutput {
    mesh: Option<Mesh>,
    height_page: HeightPage,
    anchor: DVec3,
    vertices: u64,
    triangles: u64,
    texture_bytes: u64,
    images: Option<[Image; 3]>,
    seconds: [f64; 2],
}
#[derive(Component)]
pub(super) struct SurfaceTile(TileKey);
#[derive(Resource, Default, Reflect)]
#[reflect(Resource)]
pub(super) struct SurfaceSurvey {
    pub active: bool,
    site: usize,
    #[reflect(ignore)]
    sites: Vec<([f64; 3], &'static str)>,
    orbit: Quat,
    look: Quat,
    distance: f32,
}
#[derive(Component)]
struct SurveyButton(usize);
#[derive(Component)]
struct SurveyReadout;
#[derive(Component)]
pub(super) struct TerrainBackdrop;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) struct TerrainUpdate;
pub(super) struct TerrainPlugin;
impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SurfaceSurvey>()
            .init_resource::<thessa_bevy_rcbt::CbtRenderMaterialPages>()
            .register_type::<SurfaceSurvey>()
            .add_systems(Startup, setup_terrain)
            .add_systems(PostStartup, initialize_launch_site)
            .add_systems(
                PostUpdate,
                sync_cbt_lighting.after(bevy::transform::TransformSystems::Propagate),
            )
            .add_systems(
                Update,
                survey_input.after(pilot::PilotUpdate).before(update_camera),
            )
            .add_systems(Update, update_local_sky.after(TerrainUpdate))
            .add_systems(
                Update,
                update_terrain
                    .after(update_camera)
                    .after(pilot::PilotUpdate)
                    .in_set(TerrainUpdate),
            );
    }
}
/// Share the same stellar directions, lux and ambient light as the Bevy
/// scene. A fixed shader-space sun made GPU terrain disagree with the craft.
fn sync_cbt_lighting(
    mut material: ResMut<CbtRenderMaterial>,
    ambient: Res<GlobalAmbientLight>,
    clock: Res<SimulationClock>,
    lights: Query<(&DirectionalLight, &GlobalTransform)>,
) {
    // Bound each phase in f64 before narrowing; no long-session timer drift
    // or discontinuity at an arbitrary time wrap. Pausing freezes both.
    let phases = [0.7, -0.917]
        .map(|speed| (clock.sim_seconds * speed).rem_euclid(std::f64::consts::TAU) as f32);
    if material.ocean_wave_phases != phases {
        material.ocean_wave_phases = phases;
    }
    let mut ordered: Vec<_> = lights.iter().collect();
    ordered.sort_by(|a, b| b.0.illuminance.total_cmp(&a.0.illuminance));
    let mut directions = [[0.0; 4]; 3];
    let mut colors = [[0.0; 4]; 3];
    for (index, (light, transform)) in ordered.into_iter().take(3).enumerate() {
        let direction = transform.back();
        directions[index] = [direction.x, direction.y, direction.z, 0.0];
        let color = light.color.to_linear();
        colors[index] = [
            color.red * light.illuminance,
            color.green * light.illuminance,
            color.blue * light.illuminance,
            0.0,
        ];
    }
    let color = ambient.color.to_linear();
    let ambient_lux = [
        color.red * ambient.brightness,
        color.green * ambient.brightness,
        color.blue * ambient.brightness,
        0.0,
    ];
    if material.light_directions != directions
        || material.light_colors_lux != colors
        || material.ambient_lux != ambient_lux
    {
        material.light_directions = directions;
        material.light_colors_lux = colors;
        material.ambient_lux = ambient_lux;
    }
}

fn setup_terrain(
    mut commands: Commands,
    mut survey: ResMut<SurfaceSurvey>,
    mut cbt_surface: ResMut<CbtRenderSurface>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
    graphics: Option<Res<GraphicsResolved>>,
) {
    let started = Instant::now();
    let gpu_raster = graphics
        .as_deref()
        .is_some_and(|settings| settings.0.terrain.is_gpu());
    // Hardware mesh shaders are intentionally not part of the normal client
    // path. The optional crate feature remains available for isolated adapter
    // experiments, while the game uses CPU or portable indexed raster here.
    cbt_surface.set_gpu_mesh_enabled(false);
    cbt_surface.set_gpu_raster_enabled(gpu_raster);
    info!(
        "CBT terrain raster mode: {}",
        if gpu_raster {
            "GPU indexed"
        } else {
            "CPU fallback"
        }
    );
    let albedo = load_albedo_image(&assets, "worlds/thessa-v3/albedo.png");
    let normal = load_linear_image(&assets, "worlds/thessa-v3/normal.png");
    commands.insert_resource(CbtRenderMaterial {
        albedo: albedo.clone(),
        roughness: Some(load_linear_image(&assets, "worlds/thessa-v3/roughness.png")),
        ocean: (!(std::env::var_os("THESSA_AUTOBENCH").is_some()
            && std::env::var("THESSA_AUTOBENCH_OCEAN").as_deref() == Ok("off")))
        .then(thessa_bevy_rcbt::CbtOceanMaterial::default),
        ..default()
    });
    commands.spawn((
        TerrainBackdrop,
        Mesh3d(meshes.add(Sphere::new(1.0).mesh().uv(192, 96))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(albedo),
            // Same canonical maps as the tiles so the far field behind tile
            // coverage shades continuously instead of going flat.
            normal_map_texture: Some(normal),
            metallic_roughness_texture: Some(load_linear_image(
                &assets,
                "worlds/thessa-v3/roughness.png",
            )),
            perceptual_roughness: 0.92,
            alpha_mode: AlphaMode::Opaque,
            ..default()
        })),
        Transform::default(),
        Visibility::Hidden,
        Name::new("Thessa closed terrain backdrop"),
    ));
    let system: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = system.bake().expect("ephemeris");
    // Canonical field plus deterministic bookmarks come from the shared
    // authority helper, so the headless server selects identical sites
    // without ever receiving world state.
    let (field, bookmarks) =
        thessa_flight_authority::canonical_launch_setup(&ephemeris).expect("launch sites");
    survey.sites = bookmarks
        .into_iter()
        .zip(["COAST", "HIGHLANDS", "VOLCANIC"])
        .collect();
    survey.distance = 5000.0;
    survey.orbit = Quat::IDENTITY;
    if std::env::var_os("THESSA_AUTOBENCH").is_some() {
        survey.site = std::env::var("THESSA_AUTOBENCH_SITE")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|index| *index < survey.sites.len())
            .unwrap_or(0);
        if let Some(distance) = std::env::var("THESSA_AUTOBENCH_SURVEY_DISTANCE_M")
            .ok()
            .and_then(|v| v.parse::<f32>().ok())
            .filter(|v| v.is_finite())
        {
            survey.distance = distance.clamp(80.0, 2_000_000.0);
        }
        if let Some(pitch) = std::env::var("THESSA_AUTOBENCH_SURVEY_PITCH_RAD")
            .ok()
            .and_then(|v| v.parse::<f32>().ok())
            .filter(|v| v.is_finite())
        {
            survey.orbit = Quat::from_rotation_x(pitch.clamp(-1.3, 1.3));
        }
    }
    info!(
        "world field and survey bookmarks ready in {:.3}s",
        started.elapsed().as_secs_f64()
    );
    commands.insert_resource(WorldTerrain {
        field,
        counters: default(),
        cache_bytes: 0,
        render_center: None,
        render_origin_m: DVec3::ZERO,
        cache: BTreeMap::new(),
        jobs: BTreeMap::new(),
        material_jobs: BTreeMap::new(),
        wanted: vec![],
        visible: BTreeSet::new(),
        selection_at: -1.0,
        topology_pending: false,
        selected_eye: DVec3::ZERO,
        selected_forward: DVec3::NEG_Z,
        selected_valid: false,
        eye_speed_mps: 0.0,
    });
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: px(8),
                left: percent(38),
                column_gap: px(5),
                padding: UiRect::all(px(5)),
                ..default()
            },
            ZIndex(80),
        ))
        .with_children(|row| {
            for (i, label) in ["COAST", "HIGHLANDS", "VOLCANIC", "ORBIT"]
                .iter()
                .enumerate()
            {
                row.spawn((
                    Button,
                    SurveyButton(i),
                    UiInputBlocker,
                    Node {
                        padding: UiRect::axes(px(10), px(6)),
                        border_radius: BorderRadius::all(px(4)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.025, 0.055, 0.08, 0.9)),
                ))
                .with_child((
                    Text::new(*label),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::srgb(0.7, 0.88, 0.94)),
                ));
            }
        });
    commands.spawn((
        SurveyReadout,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            bottom: px(15),
            left: px(18),
            ..default()
        },
        TextColor(Color::srgb(0.8, 0.91, 0.95)),
        ZIndex(80),
    ));
}
fn survey_input(
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Query<(&Interaction, &SurveyButton), Changed<Interaction>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    mut survey: ResMut<SurfaceSurvey>,
    mut pilot: ResMut<PilotHudState>,
) {
    let mut selected = buttons.iter().find_map(|(interaction, button)| {
        (*interaction == Interaction::Pressed).then_some(button.0)
    });
    if keys.just_pressed(KeyCode::F6) {
        selected = Some(if survey.active {
            (survey.site + 1) % 3
        } else {
            0
        });
    }
    if survey.active && keys.just_pressed(KeyCode::KeyM) {
        selected = Some(3);
    }
    if pilot.view_mode == ClientViewMode::Pilot {
        survey.active = false;
    }
    if let Some(index) = selected {
        survey.active = index < 3;
        if survey.active {
            survey.site = index;
            survey.distance = 5000.0;
            survey.look = Quat::IDENTITY;
        }
        pilot.view_mode = ClientViewMode::Map;
        // Survey is a camera mode. It must not advance or teleport the aircraft.
    }
    if survey.active {
        if mouse.pressed(MouseButton::Right) {
            let delta = Quat::from_rotation_y(-motion.delta.x * 0.004)
                * Quat::from_rotation_x(-motion.delta.y * 0.004);
            if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
                survey.look = (survey.look * delta).normalize();
            } else {
                survey.orbit = (survey.orbit * delta).normalize();
            }
        }
        for e in wheel.read() {
            survey.distance = (survey.distance * (-e.y * 0.1).exp()).clamp(80.0, 2_000_000.0);
        }
    } else {
        wheel.clear();
    }
}

fn mesh_from_tile(tile: &TerrainTile, texture_size: usize, cells: usize) -> Mesh {
    let n = cells + 1;
    let cells_f = cells as f32;
    let texture_cells = (texture_size - 3) as f32;
    let texture_size = texture_size as f32;
    let uv: Vec<_> = (0..=cells)
        .flat_map(|y| {
            (0..=cells).map(move |x| {
                [
                    (1.5 + x as f32 / cells_f * texture_cells) / texture_size,
                    (1.5 + y as f32 / cells_f * texture_cells) / texture_size,
                ]
            })
        })
        .collect();
    let mut uv = uv;
    for x in 0..cells {
        uv.push(uv[x]);
    }
    for y in 0..cells {
        uv.push(uv[y * n + cells]);
    }
    for x in (1..=cells).rev() {
        uv.push(uv[cells * n + x]);
    }
    for y in (1..=cells).rev() {
        uv.push(uv[y * n]);
    }
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, tile.positions.clone())
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, tile.normals.clone())
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uv)
    .with_inserted_indices(Indices::U32(tile.indices.clone()));
    mesh.generate_tangents().expect("terrain UVs");
    // Skirt verts duplicate edge UVs at dropped positions: zero UV gradient
    // along the skirt makes mikktspace tangents degenerate (stripes in both
    // raster normal-mapping and RT). Overwrite skirt tangents with their
    // source edge tangents instead.
    let grid = (cells + 1) * (cells + 1);
    let mut edge_grid = Vec::with_capacity(4 * cells);
    for x in 0..cells {
        edge_grid.push(x);
    }
    for y in 0..cells {
        edge_grid.push(y * (cells + 1) + cells);
    }
    for x in (1..=cells).rev() {
        edge_grid.push(cells * (cells + 1) + x);
    }
    for y in (1..=cells).rev() {
        edge_grid.push(y * (cells + 1));
    }
    if let Some(bevy::mesh::VertexAttributeValues::Float32x4(data)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_TANGENT)
        && data.len() == grid + edge_grid.len()
    {
        for (i, &src) in edge_grid.iter().enumerate() {
            data[grid + i] = data[src];
        }
    }
    mesh
}

/// CPU terrain uses an adaptive grid while keeping the graphics setting as a
/// stable mid-field baseline. Far cover is silhouette-only, the middle field
/// keeps the configured density, and the tiles that can actually carry the
/// pilot view get extra vertices for relief instead of spending the same
/// budget on the horizon.
fn mesh_cells_for_tile(base: usize, key: TileKey) -> usize {
    let base = base.clamp(8, 64);
    match key.level {
        0..=8 => (base / 3).max(8),
        9..=11 => (base * 2 / 3).max(8),
        12 => base,
        _ => (base * 2).min(64),
    }
}

// Keep existing coverage until every overlapping replacement is ready. New
// disjoint tiles can appear immediately; no parent/child meshes overlap.
fn tiles_overlap(a: TileKey, b: TileKey) -> bool {
    let (parent, child) = if a.level <= b.level { (a, b) } else { (b, a) };
    let shift = child.level - parent.level;
    parent.face == child.face && parent.x == child.x >> shift && parent.y == child.y >> shift
}

/// Partition the union of both LOD selections into complete, disjoint tiles.
/// Dropping a coarse tile after finding one fine descendant loses the rest
/// of its area. Split it along the refinement path and retain every sibling.
fn complete_terrain_cover(wanted: &[TileKey]) -> Vec<TileKey> {
    fn visit(key: TileKey, wanted: &[TileKey], cover: &mut Vec<TileKey>) {
        if wanted
            .iter()
            .any(|fine| fine.level > key.level && tiles_overlap(key, *fine))
        {
            for child in key.children() {
                visit(child, wanted, cover);
            }
        } else {
            cover.push(key);
        }
    }
    let roots: BTreeSet<_> = wanted
        .iter()
        .copied()
        .filter(|key| {
            !wanted
                .iter()
                .any(|other| other.level < key.level && tiles_overlap(*key, *other))
        })
        .collect();
    let mut cover = Vec::new();
    for root in roots {
        visit(root, wanted, &mut cover);
    }
    cover.sort();
    cover
}

fn node_is_prefix(prefix: CbtNode, node: CbtNode) -> bool {
    node.depth() >= prefix.depth() && (node.id() >> (node.depth() - prefix.depth())) == prefix.id()
}

fn cbt_candidate(node: CbtNode, action: CandidateAction, class: WorkClass) -> LeafCandidate {
    LeafCandidate {
        node,
        action,
        class,
        // The terrain adapter supplies physical projected-error candidates in
        // a later render policy. This bridge keeps requested path operations
        // deterministic; heap-id order visits ancestors before descendants.
        projected_error_px: 0.0,
        predicted_error_px: 0.0,
        time_to_needed_s: f32::INFINITY,
    }
}

/// Convert desired cube-sphere leaves to the binary path operations needed by
/// the universal CBT plugin. A quadtree level consumes two explicit Morton
/// splits; intermediate odd-depth leaves are valid topology but not complete
/// render tiles.
fn cbt_candidates_for_tiles(topology: &Tree, wanted: &[TileKey]) -> Vec<LeafCandidate> {
    let desired: Vec<_> = wanted
        .iter()
        .filter_map(|key| lod::cbt_node_for_tile(*key))
        .collect();
    let mut candidates = Vec::new();
    for target in &desired {
        let mut ancestor = *target;
        let mut path = Vec::new();
        while !topology.contains(ancestor) {
            let Some(parent) = ancestor.parent() else {
                break;
            };
            path.push(parent);
            ancestor = parent;
        }
        path.reverse();
        candidates
            .extend(path.into_iter().map(|node| {
                cbt_candidate(node, CandidateAction::Split, WorkClass::CoverageRepair)
            }));
    }

    // Reclaim binary leaves outside the requested surface cover. The overlap
    // check keeps every ancestor of a desired tile alive, including the odd
    // half-step nodes between quadtree levels.
    let leaves = topology.leaves();
    for leaf in leaves {
        let Some(parent) = leaf.parent() else {
            continue;
        };
        if parent.depth() < lod::CBT_FACE_DEPTH {
            continue;
        }
        let Some(sibling) = leaf.sibling() else {
            continue;
        };
        if !topology.contains(sibling)
            || desired
                .iter()
                .any(|target| parent != *target && node_is_prefix(parent, *target))
        {
            continue;
        }
        candidates.push(cbt_candidate(
            parent,
            CandidateAction::Merge,
            WorkClass::Cosmetic,
        ));
    }
    candidates
}

/// Display cover: retain old visible tiles overlapped only by unready
/// wanted tiles (a parent stays until ALL its children are ready — never a
/// hole, never overlap), then add ready wanted tiles that overlap nothing
/// already covered. Wanted iterates finest-first: with overlapping tiers
/// (coarse horizon cover + fine view cone) coarse keys must not shadow fine
/// children — first-fit insertion order froze the live view at coarse cover
/// (measured: 367 ready, 40 shown) because coarse sorts first.
fn ready_terrain_cover(
    wanted: &[TileKey],
    visible: &BTreeSet<TileKey>,
    ready: impl Fn(&TileKey) -> bool,
) -> BTreeSet<TileKey> {
    let mut cover: BTreeSet<_> = visible
        .iter()
        .copied()
        .filter(|old| {
            wanted
                .iter()
                .any(|new| tiles_overlap(*old, *new) && !ready(new))
        })
        .collect();
    let mut ordered: Vec<TileKey> = wanted.to_vec();
    ordered.sort_by_key(|key| std::cmp::Reverse(key.level));
    for key in ordered.iter().filter(|key| ready(key)) {
        if !cover.iter().any(|old| tiles_overlap(*old, *key)) {
            cover.insert(*key);
        }
    }
    cover
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn update_terrain(
    mut commands: Commands,
    time: Res<Time>,
    pilot: Res<PilotHudState>,
    runtime: Res<PilotFlightRuntime>,
    survey: Res<SurfaceSurvey>,
    mut world: ResMut<WorldTerrain>,
    mut cbt: CbtTerrain<'_>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut perf: ResMut<perf::PerfMonitor>,
    mut camera: Single<
        (&mut Transform, &mut Projection),
        (With<Camera3d>, Without<TerrainBackdrop>),
    >,
    mut tiles: Query<
        (Entity, &SurfaceTile, &mut Transform),
        (Without<Camera3d>, Without<TerrainBackdrop>),
    >,
    mut celestial: Query<
        &mut Visibility,
        Or<(With<CelestialVisual>, With<pilot::PilotPlanetVisual>)>,
    >,
    display: (
        Query<&mut Text, With<SurveyReadout>>,
        Query<
            (&mut Transform, &mut Visibility),
            (
                With<TerrainBackdrop>,
                Without<Camera3d>,
                Without<SurfaceTile>,
                Without<CelestialVisual>,
                Without<pilot::PilotPlanetVisual>,
            ),
        >,
    ),
    sky: Option<Res<water::WaterSky>>,
) {
    let (mut readout, mut backdrop) = display;
    let started = Instant::now();
    let terrain_mesh_cells = cbt
        .graphics
        .as_deref()
        .map(|settings| {
            if settings.0.terrain.is_gpu() {
                // The indexed CBT consumer has a fixed 33x33 GPU page and
                // index-buffer contract. CPU-only density is configurable.
                32
            } else {
                settings.0.terrain_mesh_cells as usize
            }
        })
        .unwrap_or(24);
    let active = survey.active
        || (pilot.view_mode == ClientViewMode::Pilot
            && runtime.render_terrain_origin_m().length() - world.field.params.radius_m < 80000.0);
    let gpu_raster = cbt.surface.gpu_raster_enabled();
    // The GPU raster consumer must not draw its fallback 1x1 texture over a
    // valid bootstrap sphere: that fallback is intentionally neutral grey and
    // was the source of the giant grey rectangles seen while the canonical
    // albedo was still streaming.
    let gpu_albedo_ready = !gpu_raster
        || cbt.material.as_deref().is_some_and(|material| {
            images.get(&material.albedo).is_some()
                && material
                    .roughness
                    .as_ref()
                    .is_none_or(|map| images.get(map).is_some())
        });
    // Every disjoint requested tile must be an exact resident leaf. Partial
    // overlap is not coverage: a single descendant cannot cover its parent.
    let cover_current = world.visible.len() == world.wanted.len()
        && world.wanted.iter().all(|key| world.visible.contains(key));
    let replacement_ready = !cover_current
        && gpu_raster
        && gpu_albedo_ready
        && !world.wanted.is_empty()
        && world.wanted.iter().all(|key| {
            lod::cbt_node_for_tile(*key).is_some_and(|node| {
                cbt.state.topology().contains(node) && cbt.pages.contains_page(node.id())
            })
        });
    if active && replacement_ready {
        let nodes: Vec<_> = world
            .wanted
            .iter()
            .filter_map(|key| lod::cbt_node_for_tile(*key))
            .collect();
        if cbt
            .topology
            .publish_ready_leaves(cbt.state.topology(), &nodes)
        {
            world.visible = world.wanted.iter().copied().collect();
        }
    }
    // Keep the last complete draw snapshot and its pages while workers and
    // the planning tree converge. Never flash the bootstrap on each LOD change.
    let gpu_cover_ready = gpu_raster && gpu_albedo_ready && !world.visible.is_empty();
    cbt.surface.set_gpu_surface_ready(active && gpu_cover_ready);
    world.counters.terrain_patches_generated = 0;
    if !active {
        world.render_center = None;
        for (_, mut visibility) in &mut backdrop {
            *visibility = Visibility::Hidden;
        }
        for (entity, _, _) in &mut tiles {
            commands.entity(entity).despawn();
        }
        world.visible.clear();
        world.counters.terrain_patches_visible = 0;
        world.counters.terrain_patches_wanted = 0;
        world.counters.terrain_jobs_in_flight = 0;
        world.counters.terrain_eye_speed_mps = 0;
        world.counters.terrain_detail_bias_x100 = 100;
        world.counters.terrain_lod_min = 0;
        world.counters.terrain_lod_max = 0;
        world.counters.terrain_wanted_lod_max = 0;
        world.counters.terrain_lod_histogram = [0; 18];
        world.topology_pending = false;
        for mut text in &mut readout {
            text.0.clear();
        }
        return;
    }
    let radius = world.field.params.radius_m;
    cbt.surface.set_radius_m(radius as f32);
    let (origin, rotation, eye) = if survey.active {
        let (dir, label) = survey.sites[survey.site];
        let spin = DQuat::from_rotation_y(runtime.render_spin());
        let up = spin * DVec3::from_array(dir);
        let center = up * (radius + world.field.height_m(dir, 32.0).max(0.0));
        let north = (DVec3::Y - up * up.y).normalize();
        let east = north.cross(up).normalize();
        let basis = Mat3::from_cols(east.as_vec3(), up.as_vec3(), -north.as_vec3());
        let offset =
            basis * (survey.orbit * Vec3::new(0.0, survey.distance * 0.18, survey.distance));
        let globe_eye = center + offset.as_dvec3();
        let minimum = radius
            + world
                .field
                .height_m((spin.inverse() * globe_eye.normalize()).to_array(), 32.0)
                .max(0.0)
            + 5.0;
        let globe_eye = if globe_eye.length() < minimum {
            globe_eye.normalize() * minimum
        } else {
            globe_eye
        };
        let offset = (globe_eye - center).as_vec3();
        camera.0.translation = offset;
        *camera.0 = camera.0.looking_at(Vec3::ZERO, up.as_vec3());
        camera.0.rotation *= survey.look;
        if let Projection::Perspective(p) = &mut *camera.1 {
            p.near = 1.0;
            p.far = 12_000_000.0;
        }
        // Sky and ambient stay owned by the atmosphere plugin: hardcoded
        // replacements here would fight the LUT sky and night side.
        for mut text in &mut readout {
            text.0 = format!("THESSA  /  {label}     {:.1} km", survey.distance / 1000.0);
        }
        (center, spin, spin.inverse() * (center + offset.as_dvec3()))
    } else {
        for mut text in &mut readout {
            text.0.clear();
        }
        let origin = runtime.render_terrain_origin_m();
        let spin = DQuat::from_rotation_y(runtime.render_spin());
        (
            origin,
            spin,
            spin.inverse() * (origin + camera.0.translation.as_dvec3()),
        )
    };
    cbt.surface.set_render_from_body_f64(
        bevy::math::DMat4::from_rotation_translation(rotation, -origin).to_cols_array(),
    );
    for (mut transform, mut visibility) in &mut backdrop {
        transform.translation = (-origin).as_vec3();
        transform.rotation = rotation.as_quat() * SPHERE_POLE_TO_WORLD_UP;
        transform.scale = Vec3::splat((radius - 16000.0) as f32);
        // GPU terrain uses the closed sphere while the requested cover is
        // incomplete. Keeping it behind the complete GPU cover costs a full
        // planet pass and is not part of the large_cbt-style single-surface
        // path, but hiding it after only the first page would expose holes.
        let gpu_bootstrap = gpu_raster && !gpu_cover_ready;
        let show_backdrop = if gpu_raster {
            gpu_bootstrap
        } else {
            world.visible.is_empty()
        };
        *visibility = if show_backdrop {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    if !survey.active {
        let globe_eye = rotation.inverse() * (origin + camera.0.translation.as_dvec3());
        let minimum = radius
            + world
                .field
                .height_m(globe_eye.normalize().to_array(), 32.0)
                .max(0.0)
            + 3.0;
        if globe_eye.length() < minimum {
            camera.0.translation = (rotation * globe_eye.normalize() * minimum - origin).as_vec3();
            let up = camera.0.up();
            *camera.0 = camera.0.looking_at(Vec3::ZERO, up);
        }
    }
    world.render_center = Some((-origin).as_vec3());
    world.render_origin_m = origin;
    // View frustum in the body-fixed field frame, for selection culling:
    // from 5 km the horizon cone admits a 500 km disc that would eat any
    // tile budget before the nadir refines. Non-perspective projections
    // skip culling (full cover, no holes).
    let frustum = if let Projection::Perspective(p) = &*camera.1 {
        let half_v = p.fov * 0.5;
        let half_h = (half_v.tan() * p.aspect_ratio).atan();
        // Wide swing margin: the cone must cover the frame CORNERS, not the
        // axes. A tight cone culls the ground below a forward-looking chase
        // camera (nadir is ~90° off-axis) and the pilot view never refines
        // past the coarse cover. Behind-camera tiles are still culled.
        let half_angle = f64::from(half_v.max(half_h)) + 0.6;
        let forward_body = rotation.inverse() * (camera.0.rotation * Vec3::NEG_Z).as_dvec3();
        Some(lod::SelectionFrustum {
            forward: forward_body.to_array(),
            cos_limit: half_angle.cos().clamp(-1.0, 1.0),
        })
    } else {
        None
    };
    // Camera-frame forward for the frustum and the movement trigger.
    let forward_render = (camera.0.rotation * Vec3::NEG_Z).as_dvec3();
    let forward_body = rotation.inverse() * forward_render;
    let now_s = time.elapsed_secs_f64();
    // Movement trigger (standard streaming practice): fast flight covers
    // hundreds of metres per selection period, so a pure timer always lags
    // behind the view. Reselect when the eye moved >300 m or the view swung
    // >~7°, besides the slower 0.75 s safety timer. The movement trigger
    // handles a fast craft; the timer is only a guard for a stationary camera
    // and must not run the full selection walk three times per second.
    let mut moved = !world.selected_valid;
    if world.selected_valid {
        let dt = (now_s - world.selection_at).max(1e-3);
        let eye_speed = (eye - world.selected_eye).length() / dt;
        // Smoothed: single-frame hitches must not whip the LOD bias.
        world.eye_speed_mps += (eye_speed - world.eye_speed_mps).clamp(-2000.0, 2000.0) * 0.25;
        let swing = (forward_body.normalize_or_zero() - world.selected_forward).length();
        moved = (eye - world.selected_eye).length() > 300.0 || swing > 0.12;
    }
    // Velocity detail bias (Outerra/Unreal-style): don't chase detail the
    // viewer crosses within one selection period. Hover keeps full 1/48°
    // refinement; 430 m/s cruise relaxes ~5x toward the far rule. Past
    // ~3 km/s the tile demand at full depth (≈970 new tiles/s at 500 m,
    // measured) outruns worker throughput (~200/s) by 5x, so the curve
    // keeps climbing to the LOD-internal cap of 32: at 3.7 km/s demand
    // drops to ≈174/s (500 m) and ≈10/s (5 km). Detail the viewer crosses
    // in one frame is motion-blurred anyway.
    let detail_bias = (1.0 + world.eye_speed_mps.max(0.0) / 100.0).clamp(1.0, 32.0);
    let selection_started = Instant::now();
    let mut selection_changed = false;
    if now_s - world.selection_at > 0.75 || moved || world.wanted.is_empty() {
        // Reselect when workers are nearly drained. Existing coverage is
        // retained until its overlapping replacements are ready; disjoint
        // completed tiles appear without waiting for the entire selection.
        if world.jobs.len() <= 2
            && (!gpu_raster
                || world.wanted.is_empty()
                || world.wanted.iter().all(|key| world.visible.contains(key)))
        {
            // Two-tier selection: a coarse horizon cover WITHOUT frustum
            // culling (L7, ~96 tiles) guarantees no holes ever — frustum
            // swings between selections used to cull visible regions faster
            // than the margin allowed, popping whole blocks. The frustum
            // set (L17, ~224) refines the view cone on top; overlap is
            // intended (cover logic shows the finest ready tile).
            // The cache below stays bounded (visible + wanted only).
            let mut wanted = lod::select_tiles_with_height_and_frustum(
                eye.to_array(),
                radius,
                7,
                96,
                |dir| world.field.height_m(dir, 32.0),
                None,
                detail_bias,
            );
            let mut fine = lod::select_tiles_with_height_and_frustum(
                eye.to_array(),
                radius,
                17,
                288,
                |dir| world.field.height_m(dir, 32.0),
                frustum,
                detail_bias,
            );
            wanted.append(&mut fine);
            wanted.sort();
            wanted.dedup();
            // GPU draws exact leaves; fill coarse siblings before requesting
            // pages so the render tree and page cache describe the same cover.
            world.wanted = if gpu_raster {
                complete_terrain_cover(&wanted)
            } else {
                wanted
            };
            world.selection_at = now_s;
            world.selected_eye = eye;
            world.selected_forward = forward_body.normalize_or_zero();
            world.selected_valid = true;
            world.topology_pending = true;
            selection_changed = true;
        }
    }
    perf.record_scope(
        "world.terrain_selection",
        selection_started.elapsed().as_secs_f64(),
    );
    let fov_rad = match &*camera.1 {
        Projection::Perspective(projection) => f64::from(projection.fov),
        _ => 0.0,
    };
    let render_view = RenderView {
        eye_body_m: eye.to_array(),
        forward_body: forward_body.to_array(),
        velocity_body_mps: [0.0; 3],
        fov_rad,
        pixel_error_target: 1.0,
    };
    cbt.surface.set_view_state(
        render_view.eye_body_m,
        render_view.forward_body,
        render_view.fov_rad,
    );
    let cbt_submit_started = Instant::now();
    // The CBT plugin commits only one bounded plan per PostUpdate. A large
    // wanted cover therefore needs several submissions; otherwise the first
    // partial tree was left visible forever and the GPU path mixed a few
    // active slabs with the bootstrap sphere. Rebuild candidates only while
    // convergence is pending, not on every steady-state render frame.
    if selection_changed || world.topology_pending {
        let candidates = cbt_candidates_for_tiles(cbt.state.topology(), &world.wanted);
        world.topology_pending = !candidates.is_empty();
        if world.topology_pending {
            cbt.input.submit(Some(render_view), candidates);
        }
    }
    perf.record_scope(
        "world.terrain_cbt_submit",
        cbt_submit_started.elapsed().as_secs_f64(),
    );
    let poll_started = Instant::now();
    let finished: Vec<_> = world
        .jobs
        .iter_mut()
        .filter_map(|(key, task)| block_on(poll_once(task)).map(|result| (*key, result)))
        .collect();
    perf.record_scope(
        "world.terrain_job_poll",
        poll_started.elapsed().as_secs_f64(),
    );
    let asset_upload_started = Instant::now();
    for (key, output) in finished {
        world.jobs.remove(&key);
        let TerrainBuildOutput {
            mesh: built_mesh,
            height_page,
            anchor,
            vertices,
            triangles,
            texture_bytes,
            images: built_images,
            seconds,
        } = output;
        if let Some(node) = lod::cbt_node_for_tile(key) {
            cbt.pages.set_page(node.id(), height_page);
        }
        perf.record_scope("world.terrain_meshing", seconds[0]);
        perf.record_scope("world.terrain_materials", seconds[1]);
        let (mesh, material, image_handles) = match (built_mesh, built_images) {
            (Some(built_mesh), Some([albedo_image, roughness_image, normal_image])) => {
                let mesh = meshes.add(built_mesh);
                let albedo = images.add(albedo_image);
                let roughness = images.add(roughness_image);
                let normal = images.add(normal_image);
                let material = materials.add(StandardMaterial {
                    base_color: Color::WHITE,
                    base_color_texture: Some(albedo.clone()),
                    metallic_roughness_texture: Some(roughness.clone()),
                    normal_map_texture: Some(normal.clone()),
                    // Match the Thessa/backdrop roughness so tile borders do not
                    // shade darker/lighter than the far field behind them.
                    perceptual_roughness: 0.92,
                    ..default()
                });
                (
                    Some(mesh),
                    Some(material),
                    Some([albedo, roughness, normal]),
                )
            }
            (None, None) => (None, None, None),
            _ => unreachable!("terrain output mesh/material payload must be paired"),
        };
        world.cache.insert(
            key,
            CachedTile {
                mesh,
                anchor,
                vertices,
                triangles,
                texture_bytes,
                material,
                images: image_handles,
            },
        );
        world.counters.terrain_patches_generated += 1;
    }
    perf.record_scope(
        "world.terrain_asset_upload",
        asset_upload_started.elapsed().as_secs_f64(),
    );
    // Build order: coarse cover (L7-) first so holes close immediately,
    // then nearest-first for detail. Key order is arbitrary — without this
    // the near field waits behind hundreds of far tiles, and without the
    // coarse-first rule a fresh selection shows sky through missing cover.
    let mut pending: Vec<_> = world
        .wanted
        .iter()
        .filter(|key| !world.cache.contains_key(key) && !world.jobs.contains_key(key))
        .copied()
        .collect();
    pending.sort_by_cached_key(|key| {
        fn priority(
            field: &thessa_worldgen_rocky::field::PlanetField,
            eye: DVec3,
            radius: f64,
            key: TileKey,
        ) -> f64 {
            let dir = key.direction(0.5, 0.5);
            let surface = radius + field.height_m(dir, 32.0).max(0.0);
            let dist = (eye - DVec3::from_array(dir.map(|v| v * surface)))
                .length()
                .max(1.0);
            key.span_m(radius) / dist
        }
        // Positive finite priorities: integer bit order matches float order.
        // Coarse cover sorts before everything (level is the major key).
        (
            key.level.min(8),
            std::cmp::Reverse(priority(&world.field, eye, radius, *key).to_bits()),
        )
    });
    // Eight in flight on an 8-core box: tile builds are CPU-bound
    // field sampling, and the pool is core-sized.
    let pending: Vec<_> = pending
        .into_iter()
        .take(8_usize.saturating_sub(world.jobs.len()))
        .collect();
    for key in pending {
        let field = world.field.clone();
        let tile_mesh_cells = if gpu_raster {
            32
        } else {
            mesh_cells_for_tile(terrain_mesh_cells, key)
        };
        world.jobs.insert(
            key,
            AsyncComputeTaskPool::get().spawn(async move {
                let start = Instant::now();
                // The direct GPU path needs only the quantized height page.
                // Do not build CPU positions, normals, skirts, or material
                // images for a surface that will never create a Bevy entity.
                let (
                    height_page,
                    anchor,
                    vertices,
                    triangles,
                    mesh,
                    images,
                    texture_bytes,
                    mesh_s,
                    material_s,
                ) = if gpu_raster {
                    let page = lod::bake_height_page(&field, key, 33, 0.5)
                        .expect("GPU terrain height page must fit its error budget");
                    (
                        page,
                        DVec3::ZERO,
                        0,
                        0,
                        None,
                        None,
                        0,
                        start.elapsed().as_secs_f64(),
                        0.0,
                    )
                } else {
                    let tile = lod::build_tile(&field, key, tile_mesh_cells);
                    let mesh_s = start.elapsed().as_secs_f64();
                    let texture_start = Instant::now();
                    let texture = lod::build_surface_texture_for_mesh(
                        &field,
                        key,
                        lod::texture_cells_for_level(key.level),
                        tile_mesh_cells,
                    );
                    let material_s = texture_start.elapsed().as_secs_f64();
                    // Mesh assembly (tangents) and image upload prep (mipmaps:
                    // ~65k sRGB powf per 128 px tile) stay on the pool: per
                    // finished tile the frame thread only inserts handles.
                    let mesh = mesh_from_tile(&tile, texture.size, tile_mesh_cells);
                    let images = [
                        surface_image(texture.size, texture.albedo.clone(), true),
                        surface_image(texture.size, texture.roughness.clone(), false),
                        surface_image(texture.size, texture.normal.clone(), false),
                    ];
                    (
                        tile.height_page,
                        DVec3::from_array(tile.anchor_m),
                        tile.positions.len() as u64,
                        (tile.indices.len() / 3) as u64,
                        Some(mesh),
                        Some(images),
                        mip_bytes(texture.size) * 3,
                        mesh_s,
                        material_s,
                    )
                };
                TerrainBuildOutput {
                    mesh,
                    height_page,
                    anchor,
                    vertices,
                    triangles,
                    texture_bytes,
                    images,
                    seconds: [mesh_s, material_s],
                }
            }),
        );
        world.counters.terrain_cache_misses += 1;
    }
    // Material refinement has its own two-worker budget. Height/cover jobs
    // never wait for it; a resident canonical globe map remains the fallback.
    if gpu_raster {
        let finished: Vec<_> = world
            .material_jobs
            .iter_mut()
            .filter_map(|(key, task)| block_on(poll_once(task)).map(|page| (*key, page)))
            .collect();
        for (key, page) in finished {
            world.material_jobs.remove(&key);
            if world.cache.contains_key(&key)
                && let Some(node) = lod::cbt_node_for_tile(key)
            {
                cbt.material_pages.set_page(node.id(), page);
            }
        }
        let mut material_wanted: Vec<_> = world.visible.iter().copied().collect();
        material_wanted.sort_by_key(|key| {
            (
                std::cmp::Reverse(key.level),
                lod::cbt_node_for_tile(*key).map_or(0, |node| node.id()),
            )
        });
        material_wanted.truncate(256);
        for key in material_wanted {
            if world.material_jobs.len() >= 2 {
                break;
            }
            let Some(node) = lod::cbt_node_for_tile(key) else {
                continue;
            };
            if cbt.material_pages.contains_page(node.id()) || world.material_jobs.contains_key(&key)
            {
                continue;
            }
            let field = world.field.clone();
            world.material_jobs.insert(
                key,
                AsyncComputeTaskPool::get().spawn(async move {
                    let page = lod::build_gpu_material_page(&field, key);
                    thessa_bevy_rcbt::CbtMaterialPage::from_rgba8(page.rgba)
                        .expect("fixed material page layout")
                }),
            );
        }
    }
    let entity_sync_started = Instant::now();
    if !gpu_raster && (selection_changed || world.counters.terrain_patches_generated > 0) {
        let desired = ready_terrain_cover(&world.wanted, &world.visible, |key| {
            world.cache.contains_key(key)
        });
        if desired != world.visible {
            for (entity, tile, _) in &mut tiles {
                if !desired.contains(&tile.0) {
                    commands.entity(entity).despawn();
                }
            }
            for key in desired.difference(&world.visible) {
                let tile = &world.cache[key];
                if cbt.surface.gpu_raster_enabled() {
                    continue;
                }
                let (Some(mesh), Some(material)) = (&tile.mesh, &tile.material) else {
                    continue;
                };
                let tile_entity = commands.spawn((
                    SurfaceTile(*key),
                    Mesh3d(mesh.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_translation((rotation * tile.anchor - origin).as_vec3())
                        .with_rotation(rotation.as_quat()),
                    Name::new(format!("Thessa tile {key:?}")),
                ));
                // Shared sky probe at spawn (atomic with creation: no
                // query/insert race when cover churns under warp).
                if let Some(sky) = sky.as_deref() {
                    // The probe must not share the mesh entity's transform:
                    // scaling that transform would scale the terrain. Its
                    // child uses the tile's physical cube-sphere span, with
                    // a conservative margin for the apron and height field.
                    let tile_entity = tile_entity.id();
                    let probe_extent_m = key.span_m(world.field.params.radius_m) * 1.5;
                    commands.spawn((
                        water::tile_water_probe(sky),
                        Transform::from_scale(Vec3::splat(probe_extent_m as f32)),
                        ChildOf(tile_entity),
                    ));
                }
            }
            world.counters.terrain_cache_hits += desired.len() as u64;
            world.visible = desired;
        }
    }
    // The closed sphere is only a bootstrap/GPU fallback. Keeping it visible
    // behind CPU tiles makes incomplete cover show a second, low-detail
    // coastline and produces large stepped land/water transitions.
    // In GPU mode `visible` tracks the retained complete draw snapshot,
    // independently of the planning tree and its in-flight replacement.
    let gpu_bootstrap = gpu_raster && !gpu_cover_ready;
    for (_, mut visibility) in &mut backdrop {
        let show_backdrop = if gpu_raster {
            gpu_bootstrap
        } else {
            world.visible.is_empty()
        };
        *visibility = if show_backdrop {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    for (_, tile, mut transform) in &mut tiles {
        if let Some(cached) = world.cache.get(&tile.0) {
            transform.translation = (rotation * cached.anchor - origin).as_vec3();
            transform.rotation = rotation.as_quat();
        }
    }
    perf.record_scope(
        "world.terrain_entity_sync",
        entity_sync_started.elapsed().as_secs_f64(),
    );
    let terrain_surface_ready = if gpu_raster {
        gpu_cover_ready
    } else {
        !world.visible.is_empty()
    };
    if terrain_surface_ready {
        for mut visibility in &mut celestial {
            *visibility = Visibility::Hidden;
        }
    }
    // Bounded cache, retaining visible and in-flight selection only.
    if world.cache.len() > 512 {
        let stale: Vec<_> = world
            .cache
            .keys()
            .filter(|k| !world.visible.contains(k) && !world.wanted.contains(k))
            .copied()
            .collect();
        for key in stale {
            if let Some(tile) = world.cache.remove(&key) {
                if let Some(node) = lod::cbt_node_for_tile(key) {
                    cbt.pages.remove_page(node.id());
                    cbt.material_pages.remove_page(node.id());
                }
                if let Some(mesh) = tile.mesh {
                    meshes.remove(mesh.id());
                }
                if let Some(material) = tile.material {
                    materials.remove(material.id());
                }
                if let Some(tile_images) = tile.images {
                    for image in tile_images {
                        images.remove(image.id());
                    }
                }
            }
        }
    }
    let counters_started = Instant::now();
    world.counters.assets_loaded = world.cache.len() as u32 * 5;
    world.counters.assets_pending = world.jobs.len() as u32 * 5;
    world.counters.terrain_patches_visible = world.visible.len() as u32;
    world.counters.terrain_patches_wanted = world.wanted.len() as u32;
    world.counters.terrain_jobs_in_flight = world.jobs.len() as u32;
    world.counters.terrain_eye_speed_mps = world.eye_speed_mps.round().max(0.0) as u32;
    world.counters.terrain_detail_bias_x100 = (detail_bias * 100.0).round() as u32;
    let mut visible_lod_histogram = [0_u32; 18];
    for key in &world.visible {
        visible_lod_histogram[usize::from(key.level.min(17))] += 1;
    }
    world.counters.terrain_lod_histogram = visible_lod_histogram;
    world.counters.terrain_lod_min = world
        .visible
        .iter()
        .map(|key| u32::from(key.level))
        .min()
        .unwrap_or(0);
    world.counters.terrain_lod_max = world
        .visible
        .iter()
        .map(|key| u32::from(key.level))
        .max()
        .unwrap_or(0);
    world.counters.terrain_wanted_lod_max = world
        .wanted
        .iter()
        .map(|key| u32::from(key.level))
        .max()
        .unwrap_or(0);
    world.counters.streaming_queued = world
        .wanted
        .iter()
        .filter(|k| !world.cache.contains_key(k))
        .count() as u32;
    world.counters.terrain_vertices = world
        .visible
        .iter()
        .filter_map(|k| world.cache.get(k))
        .map(|t| t.vertices)
        .sum();
    world.counters.terrain_triangles = world
        .visible
        .iter()
        .filter_map(|k| world.cache.get(k))
        .map(|t| t.triangles)
        .sum();
    world.cache_bytes = world
        .cache
        .values()
        .map(|t| t.vertices * 48 + t.triangles * 12 + t.texture_bytes)
        .sum::<u64>()
        + cbt.material_pages.byte_len() as u64;
    perf.record_scope(
        "world.terrain_counters",
        counters_started.elapsed().as_secs_f64(),
    );
    perf.record_scope("world.streaming", started.elapsed().as_secs_f64());
}

fn mip_bytes(mut size: usize) -> u64 {
    let mut bytes = 0;
    loop {
        bytes += (size * size * 4) as u64;
        if size == 1 {
            return bytes;
        }
        size /= 2;
    }
}

fn surface_image(size: usize, pixels: Vec<u8>, srgb: bool) -> Image {
    use bevy::{
        image::{ImageSampler, ImageSamplerDescriptor},
        render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    };
    // Exact sRGB decode table: `(v/255)^2.2` evaluated once per byte value
    // instead of per texel per mip level. Bitwise identical — the table
    // memoizes the same expression, it does not approximate it.
    static SRGB_TO_LINEAR: std::sync::OnceLock<[f32; 256]> = std::sync::OnceLock::new();
    let decode =
        SRGB_TO_LINEAR.get_or_init(|| std::array::from_fn(|i| (i as f32 / 255.0).powf(2.2)));
    let mut data = pixels.clone();
    let mut previous = pixels;
    let mut width = size;
    let mut levels = 1;
    while width > 1 {
        let next_width = (width / 2).max(1);
        let mut next = vec![0_u8; next_width * next_width * 4];
        for y in 0..next_width {
            for x in 0..next_width {
                for channel in 0..4 {
                    let mut sum = 0.0_f32;
                    for dy in 0..2 {
                        for dx in 0..2 {
                            let byte = previous[((y * 2 + dy).min(width - 1) * width
                                + (x * 2 + dx).min(width - 1))
                                * 4
                                + channel];
                            sum += if srgb && channel < 3 {
                                decode[byte as usize]
                            } else {
                                byte as f32 / 255.0
                            };
                        }
                    }
                    let mean = sum / 4.0;
                    next[(y * next_width + x) * 4 + channel] = ((if srgb && channel < 3 {
                        mean.powf(1.0 / 2.2)
                    } else {
                        mean
                    }) * 255.0)
                        .round()
                        as u8;
                }
            }
        }
        data.extend_from_slice(&next);
        previous = next;
        width = next_width;
        levels += 1;
    }
    let mut image = Image::new(
        Extent3d {
            width: size as u32,
            height: size as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data[..size * size * 4].to_vec(),
        if srgb {
            TextureFormat::Rgba8UnormSrgb
        } else {
            TextureFormat::Rgba8Unorm
        },
        RenderAssetUsages::RENDER_WORLD,
    );
    image.data = Some(data);
    image.texture_descriptor.mip_level_count = levels;
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        anisotropy_clamp: 8,
        ..ImageSamplerDescriptor::linear()
    });
    image
}

fn initialize_launch_site(
    world: Res<WorldTerrain>,
    survey: Res<SurfaceSurvey>,
    ephemeris: Res<RuntimeEphemeris>,
    mut flight: ResMut<PilotFlightRuntime>,
) {
    flight.initialize_world_site(world.field.clone(), survey.sites[0].0, &ephemeris.ephemeris);
}

/// Distant body proxies retain ephemeris directions and angular sizes. They
/// sit behind local terrain in the same depth buffer, so the horizon occludes
/// them instead of drawing celestial objects through the ground.
#[allow(clippy::too_many_arguments)]
fn update_local_sky(
    mut commands: Commands,
    clock: Res<SimulationClock>,
    world: Res<WorldTerrain>,
    flight: Res<PilotFlightRuntime>,
    hud: Res<PilotHudState>,
    survey: Res<SurfaceSurvey>,
    ephemeris: Res<RuntimeEphemeris>,
    stars: Option<Res<super::atmosphere::StarBodyIds>>,
    camera: Single<&Transform, (With<Camera3d>, Without<CelestialVisual>)>,
    mut bodies: Query<
        (Entity, &CelestialVisual, &mut Transform, &mut Visibility),
        Without<Camera3d>,
    >,
) {
    if !survey.active && hud.view_mode != ClientViewMode::Pilot {
        return;
    }
    let time = SimTime(clock.sim_seconds);
    let reference = ephemeris
        .ephemeris
        .body_state(flight.reference_body, time)
        .expect("observer body");
    let local_origin = if survey.active {
        world.render_origin_m
    } else {
        flight.render_terrain_origin_m()
    } + camera.translation.as_dvec3();
    let observer =
        reference.position_inertial + DVec3::new(local_origin.x, -local_origin.z, local_origin.y);
    const SKY_DISTANCE: f64 = 6_000_000.0;
    for (entity, visual, mut transform, mut visibility) in &mut bodies {
        commands
            .entity(entity)
            .remove::<bevy::solari::prelude::RaytracingMesh3d>();
        let body = ephemeris.ephemeris.body(visual.id).expect("sky body");
        // Star meshes stay hidden: their disks come from the atmosphere
        // shader via directional lights. Membership is config data, so any
        // star list works without name matching.
        let is_star = stars
            .as_deref()
            .is_some_and(|stars| stars.ids.contains(&visual.id));
        if visual.id == flight.reference_body || is_star {
            *visibility = Visibility::Hidden;
            continue;
        }
        let state = ephemeris
            .ephemeris
            .body_state(visual.id, time)
            .expect("sky ephemeris");
        let delta = state.position_inertial - observer;
        let distance = delta.length();
        let dir = delta / distance;
        let angular_radius = body.radius_m / distance;
        transform.translation = camera.translation
            + Vec3::new(dir.x as f32, dir.z as f32, -dir.y as f32) * SKY_DISTANCE as f32;
        transform.scale = Vec3::splat((angular_radius * SKY_DISTANCE) as f32);
        transform.rotation = visual_rotation(body, time);
        *visibility = if angular_radius > 0.00005 {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

#[cfg(test)]
mod streaming_tests {
    use super::*;
    use thessa_rcbt_core::{FrameBudget, plan_frame};

    #[test]
    fn mixed_lod_cover_preserves_parent_area_and_fine_detail() {
        let root = TileKey::root(0);
        let fine = root.children()[0].children()[3];
        let cover = complete_terrain_cover(&[root, fine]);
        assert!(cover.contains(&fine));
        assert_eq!(cover.len(), 7);
        let area: f64 = cover
            .iter()
            .map(|key| 4.0_f64.powi(-(key.level as i32)))
            .sum();
        assert_eq!(area, 1.0);
        for (i, a) in cover.iter().enumerate() {
            assert!(cover[i + 1..].iter().all(|b| !tiles_overlap(*a, *b)));
        }
        assert_eq!(complete_terrain_cover(&cover), cover);
    }

    #[test]
    fn cbt_cover_converges_after_refining_and_coarsening() {
        let root = TileKey::root(0);
        let fine = root.children()[0].children()[3];
        let mut tree = Tree::at_depth(12, lod::CBT_FACE_DEPTH).unwrap();
        for wanted in [complete_terrain_cover(&[root, fine]), vec![root]] {
            for _ in 0..64 {
                let candidates = cbt_candidates_for_tiles(&tree, &wanted);
                if candidates.is_empty() {
                    break;
                }
                let plan = plan_frame(&tree, candidates, FrameBudget { max_operations: 2 });
                tree.apply_batch(plan.updates()).unwrap();
            }
            assert!(cbt_candidates_for_tiles(&tree, &wanted).is_empty());
            assert!(
                wanted
                    .iter()
                    .all(|key| tree.contains(lod::cbt_node_for_tile(*key).unwrap()))
            );
        }
    }

    #[test]
    fn refinement_retains_parent_until_all_children_are_ready() {
        let parent = TileKey::root(0);
        let children = parent.children();
        let visible = BTreeSet::from([parent]);
        assert_eq!(
            ready_terrain_cover(&children, &visible, |key| *key != children[3]),
            visible
        );
        assert_eq!(
            ready_terrain_cover(&children, &visible, |_| true),
            BTreeSet::from(children)
        );
    }

    #[test]
    fn disjoint_tiles_appear_while_other_tiles_are_loading() {
        let wanted = [TileKey::root(0), TileKey::root(1)];
        assert_eq!(
            ready_terrain_cover(&wanted, &BTreeSet::new(), |key| *key == wanted[0]),
            BTreeSet::from([wanted[0]])
        );
    }

    #[test]
    fn coarsening_replaces_children_together_without_overlapping_geometry() {
        let parent = TileKey::root(2);
        let visible = BTreeSet::from(parent.children());
        assert_eq!(ready_terrain_cover(&[parent], &visible, |_| false), visible);
        assert_eq!(
            ready_terrain_cover(&[parent], &visible, |_| true),
            BTreeSet::from([parent])
        );
    }

    #[test]
    fn cbt_bridge_plans_ancestors_before_descendants() {
        let topology = Tree::at_depth(12, lod::CBT_FACE_DEPTH).unwrap();
        let wanted = [TileKey {
            face: 0,
            level: 2,
            x: 0,
            y: 0,
        }];
        let candidates = cbt_candidates_for_tiles(&topology, &wanted);
        let plan = plan_frame(&topology, candidates, FrameBudget { max_operations: 4 });
        assert_eq!(plan.updates().len(), 4);
        assert_eq!(
            plan.updates()[0],
            thessa_rcbt_core::Update::Split(CbtNode::new(8, 3).unwrap())
        );
    }
}
