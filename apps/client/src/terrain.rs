//! Shared canonical world field, bounded async cube-sphere terrain and local survey.
use super::*;
use bevy::{
    asset::RenderAssetUsages,
    ecs::system::SystemParam,
    math::{DQuat, DVec3},
    mesh::{Indices, PrimitiveTopology},
};
use std::{collections::BTreeSet, sync::Arc, time::Instant};
use thessa_bevy_rcbt::{
    CbtCloudShadow, CbtFrameInput, CbtNightLights, CbtRenderMaterial, CbtRenderPages,
    CbtRenderState, CbtRenderSurface, CbtRenderTopology, RenderView,
};
use thessa_rcbt_core::{CandidateAction, LeafCandidate, Node as CbtNode, Tree, WorkClass};
use thessa_render_core::BodyFrame;
#[cfg(test)]
use thessa_terrain_stream::selection::{self, complete_terrain_cover};
use thessa_terrain_stream::{
    JobHandle,
    height::{HeightBuild, HeightPolicy, HeightStream},
    material::{MaterialDemand, MaterialStream},
    selection::{SelectionHistory, SelectionPolicy, SelectionView, tiles_overlap},
};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TerrainTile, TileKey},
};

#[derive(SystemParam)]
struct CbtTerrain<'w> {
    state: Res<'w, CbtRenderState>,
    presentation: Res<'w, thessa_bevy_rcbt::CbtGpuPresentation>,
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
    heights: HeightStream<CachedTile, TerrainBuildOutput>,
    height_capacity_rejected: bool,
    material_stream: MaterialStream,
    wanted: Vec<TileKey>,
    fine_selection: Vec<TileKey>,
    selection_epoch: u64,
    cover_dirty: bool,
    visible: BTreeSet<TileKey>,
    selection_history: SelectionHistory,
    /// A bounded CBT plan may take several PostUpdate frames to converge.
    topology_pending: bool,
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
    #[reflect(ignore)]
    pub(super) camera: thessa_flight_authority::survey::SurveyCamera,
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
            .add_systems(
                Startup,
                (resolve_terrain_capabilities, setup_terrain).chain(),
            )
            .add_systems(PostStartup, initialize_launch_site)
            .add_systems(
                PostUpdate,
                sync_cbt_lighting.after(bevy::transform::TransformSystems::Propagate),
            )
            .add_systems(PostUpdate, sync_cbt_effects.after(sync_cbt_lighting))
            .add_systems(Update, update_tile_cloud_shadows.in_set(TerrainUpdate))
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

/// Cloud-deck shadows and night city lights for the GPU terrain surface.
/// Same deck math as the shell follower (shared helpers in `beauty`), packed
/// as uniforms; the fragment shader samples the deck bake the shells show.
/// Disabled flags pack as zeros (static layout, no shader variants) and the
/// texture binds fall back to the albedo image.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn sync_cbt_effects(
    mut material: ResMut<CbtRenderMaterial>,
    graphics: Option<Res<GraphicsResolved>>,
    beauty: Option<Res<beauty::BeautyImages>>,
    night: Option<Res<crate::night_lights::NightLights>>,
    clock: Option<Res<SimulationClock>>,
    terrain: Option<Res<WorldTerrain>>,
    sun: Query<&GlobalTransform, With<PrimaryStarLight>>,
    visuals: Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<beauty::CloudShell>,
            Without<beauty::AuroraShell>,
            Without<beauty::CloudVolume>,
        ),
    >,
) {
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    let t = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);
    // The entity match needs no terrain; the live fallback does.
    let planet = beauty::planet_frame_for_body("thessa", &visuals, terrain.as_deref(), t);
    let sun_dir = sun
        .single()
        .map(|tf| {
            let d = tf.back();
            Vec3::new(d.x, d.y, d.z)
        })
        .unwrap_or(Vec3::Y);
    let spin = beauty::deck_spin_rad_s("thessa", 0).unwrap_or(5.0e-6);
    let cloud_shadow = match (&beauty, &planet) {
        (Some(images), Some(planet)) if r.clouds_cast_shadows => {
            let relative =
                beauty::shadow_relative_rotation(planet.rotation, spin, t, r.clouds_animate);
            let sun_body = beauty::shadow_sun_body(planet.rotation, sun_dir);
            Some(CbtCloudShadow {
                texture: images.cloud_low.clone(),
                relative_rotation: relative.to_array(),
                sun_body_dir: [sun_body.x, sun_body.y, sun_body.z, 0.0],
                deck_height_m: beauty::CLOUD_SHADOW_DECK_HEIGHT_M as f32,
                strength: beauty::CLOUD_SHADOW_STRENGTH,
            })
        }
        _ => None,
    };
    let night_lights = match &night {
        Some(lights) if r.night_lights_enabled => Some(CbtNightLights {
            texture: lights.image.clone(),
            intensity: r.night_lights_intensity,
        }),
        _ => None,
    };
    if material.cloud_shadow != cloud_shadow {
        material.cloud_shadow = cloud_shadow;
    }
    if material.night_lights != night_lights {
        material.night_lights = night_lights;
    }
}

/// Cloud-deck shadows for CPU-streamed tiles: the same deck bake sampled on
/// the CPU, applied as a per-tile base-color factor at 5 Hz. The GPU surface
/// samples in-shader (`sync_cbt_effects`); this covers the CPU fallback path
/// with no shader change. Skipped entirely when the flag is off.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn update_tile_cloud_shadows(
    time: Res<Time>,
    mut acc: Local<f32>,
    graphics: Option<Res<GraphicsResolved>>,
    beauty: Option<Res<beauty::BeautyImages>>,
    images: Res<Assets<Image>>,
    clock: Option<Res<SimulationClock>>,
    sun: Query<&GlobalTransform, With<PrimaryStarLight>>,
    visuals: Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<beauty::CloudShell>,
            Without<beauty::AuroraShell>,
            Without<beauty::CloudVolume>,
        ),
    >,
    tiles: Query<(
        &SurfaceTile,
        &GlobalTransform,
        &MeshMaterial3d<StandardMaterial>,
    )>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    *acc += time.delta_secs();
    if *acc < 0.2 {
        return;
    }
    *acc = 0.0;
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    if !r.clouds_cast_shadows {
        return;
    }
    // CPU tiles only: the GPU surface shades the same deck per-pixel
    // in-shader (`sync_cbt_effects`). Running both double-dims, and the
    // per-tile constant prints hard tile-edged quads over the smooth field
    // (screenshot-caught on orbit).
    if r.terrain.is_gpu() {
        return;
    }
    let (Some(images_res), Some(clock)) = (beauty.as_deref(), clock.as_deref()) else {
        return;
    };
    let t = clock.sim_seconds;
    let Some(planet) = beauty::planet_frame_for_body("thessa", &visuals, None, t) else {
        return;
    };
    let Ok(sun_tf) = sun.single() else {
        return;
    };
    let back = sun_tf.back();
    let sun_dir = Vec3::new(back.x, back.y, back.z);
    let Some(cloud) = images.get(&images_res.cloud_low) else {
        return;
    };
    let (Some(data), w, h) = (cloud.data.as_ref(), cloud.width(), cloud.height()) else {
        return;
    };
    let spin = beauty::deck_spin_rad_s("thessa", 0).unwrap_or(5.0e-6);
    let shell_rot = beauty::deck_world_rotation(planet.rotation, spin, t, r.clouds_animate);
    let inv_shell = shell_rot.inverse();
    // Planet scale carries the body radius (entity match and live fallback
    // both scale by it), so no terrain field is needed here.
    let radius = planet.scale.x.max(1.0);
    for (_, transform, mat3d) in &tiles {
        let up = (transform.translation() - planet.translation).normalize_or_zero();
        if up.length_squared() < 0.5 {
            continue;
        }
        let sample = beauty::shadow_sample_dir(
            up,
            sun_dir,
            radius,
            beauty::CLOUD_SHADOW_DECK_HEIGHT_M as f32,
        );
        let alpha = beauty::cloud_alpha_at(data, w, h, beauty::shell_mesh_uv(inv_shell * sample));
        let f = beauty::shadow_factor_from_alpha(alpha);
        if let Some(mut mat) = materials.get_mut(&mat3d.0) {
            mat.base_color = Color::srgb(f, f, f);
        }
    }
}

fn resolve_terrain_capabilities(
    device: Option<Res<bevy::render::renderer::RenderDevice>>,
    adapter: Option<Res<bevy::render::renderer::RenderAdapter>>,
    requested: Option<Res<GraphicsRequested>>,
    graphics: Option<ResMut<GraphicsResolved>>,
) {
    let (Some(device), Some(requested), Some(mut graphics)) = (device, requested, graphics) else {
        return;
    };
    let is_discrete = adapter
        .as_ref()
        .map(|a| a.get_info().device_type == wgpu::DeviceType::DiscreteGpu);
    let ray_query = Some(
        device
            .features()
            .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY),
    );
    let caps = Capabilities {
        // Always available: without the `mesh-shaders` crate feature this
        // reports false and terrain resolves to the indexed path.
        terrain_mesh_supported: Some(thessa_bevy_rcbt::mesh_shader_supported(&device)),
        ray_query_supported: ray_query,
        is_discrete_gpu: is_discrete,
    };
    let resolved = ResolvedGraphicsSettings::from_requested(&requested.0, &caps);
    graphics.0.terrain = resolved.terrain;
    graphics.0.ray_tracing = resolved.ray_tracing;
    graphics.0.notes.retain(|note| {
        !note.starts_with("terrain mesh capability") && !note.starts_with("auto: ray-query")
    });
    graphics
        .0
        .notes
        .extend(resolved.notes.into_iter().filter(|note| {
            note.starts_with("terrain mesh capability") || note.starts_with("auto: ray-query")
        }));
}

/// Loud startup check for the canonical terrain maps. The tile raster,
/// the backdrop sphere and the globe material all consume the same three
/// `worlds/thessa-v10` images; a missing file leaves a pending asset handle
/// which silently skips every GPU draw, so missing files must scream here
/// instead of presenting an empty planet.
fn check_terrain_map_assets() {
    let dir = std::path::PathBuf::from(client_asset_directory());
    for rel in [
        "worlds/thessa-v10/manifest.json",
        "worlds/thessa-v10/albedo.png",
        "worlds/thessa-v10/normal.png",
        "worlds/thessa-v10/roughness.png",
    ] {
        if !dir.join(rel).is_file() {
            error!(
                "[terrain] required map missing: {} (GPU terrain falls back to globe/empty; check packaged assets)",
                dir.join(rel).display()
            );
        }
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
    cbt_surface.set_gpu_mesh_enabled(graphics.as_deref().is_some_and(|settings| {
        settings.0.terrain == thessa_graphics::ResolvedTerrainRender::GpuMesh
    }));
    cbt_surface.set_gpu_raster_enabled(gpu_raster);
    // Resolved mode at startup. The mesh path re-checks dispatch limits per
    // frame against the real leaf count (see bevy-rcbt `draw_cbt_geometry`,
    // which logs when it falls back); a `gpu_mesh` line here means "mesh
    // attempted, indexed fallback when dispatch limits exceed", never a
    // promise that every frame draws mesh.
    info!(
        "CBT terrain raster mode: {}",
        graphics
            .as_deref()
            .map_or("cpu", |settings| settings.0.terrain.as_str())
    );
    check_terrain_map_assets();
    let albedo = load_albedo_image(&assets, "worlds/thessa-v10/albedo.png");
    let normal = load_linear_image(&assets, "worlds/thessa-v10/normal.png");
    commands.insert_resource(CbtRenderMaterial {
        albedo: albedo.clone(),
        roughness: Some(load_linear_image(
            &assets,
            "worlds/thessa-v10/roughness.png",
        )),
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
                "worlds/thessa-v10/roughness.png",
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
        .zip(thessa_flight_authority::SURVEY_SITE_LABELS)
        .collect();
    survey.camera = thessa_flight_authority::survey::SurveyCamera::survey_default();
    if std::env::var_os("THESSA_AUTOBENCH").is_some()
        || std::env::var_os("THESSA_SCREENSHOT_PATH").is_some()
    {
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
            survey.camera.set_distance_m(distance);
        }
        if let Some(pitch) = std::env::var("THESSA_AUTOBENCH_SURVEY_PITCH_RAD")
            .ok()
            .and_then(|v| v.parse::<f32>().ok())
            .filter(|v| v.is_finite())
        {
            survey
                .camera
                .set_orbit(Quat::from_rotation_x(pitch.clamp(-1.3, 1.3)));
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
        heights: HeightStream::default(),
        height_capacity_rejected: false,
        material_stream: MaterialStream::new(1024),
        wanted: vec![],
        fine_selection: Vec::new(),
        selection_epoch: 0,
        cover_dirty: true,
        visible: BTreeSet::new(),
        selection_history: SelectionHistory::default(),
        topology_pending: false,
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
                    // No `Hovered` tracker: survey buttons have no hover tint;
                    // press edges arrive via `Added<Pressed>` below.
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
    buttons: Query<&SurveyButton, (With<Button>, Added<Pressed>)>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    mut survey: ResMut<SurfaceSurvey>,
    mut pilot: ResMut<PilotHudState>,
) {
    if std::env::var_os("THESSA_AUTOBENCH").is_some()
        && std::env::var_os("THESSA_AUTOBENCH_STATIC").is_some()
    {
        wheel.clear();
        return;
    }
    let mut selected = buttons.iter().map(|button| button.0).next();
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
            survey.camera.reset_site();
        }
        pilot.view_mode = ClientViewMode::Map;
        // Survey is a camera mode. It must not advance or teleport the aircraft.
    }
    if survey.active {
        if mouse.pressed(MouseButton::Right) {
            survey.camera.drag(
                motion.delta.x,
                motion.delta.y,
                keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight),
            );
        }
        for e in wheel.read() {
            survey.camera.zoom(e.y);
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

// ---------------------------------------------------------------------------
// Terrain dataflow stages.
//
// `update_terrain` below is only the wiring between these stages:
// selection -> requests -> streaming -> ready pages -> presentation, with the
// CPU-mesh fallback isolated in `sync_tile_entities`. Each stage takes narrow
// borrows (a `&CbtRenderPages` here does not trip Bevy change detection, a
// `&mut WorldTerrain` there does not alias the render snapshot), so the
// stages can move one by one from the Bevy schedule into a dedicated
// scheduler without untangling a mega-system again.
//

/// CPU-fallback half of a finished build: turn pool-built mesh/images into
/// asset handles and return the opaque resident payload. The shared height
/// lifecycle inserts it on both paths (GPU payloads have no asset handles).
/// Only the `Assets`/`Handle` creation is CPU-path-only. This is
/// the only place (besides eviction) allowed to touch
/// `Assets<Mesh/Image/StandardMaterial>`.
#[allow(clippy::too_many_arguments)]
fn present_cpu_tile(
    meshes: &mut Assets<Mesh>,
    images: &mut Assets<Image>,
    materials: &mut Assets<StandardMaterial>,
    built_mesh: Option<Mesh>,
    built_images: Option<[Image; 3]>,
    anchor: DVec3,
    vertices: u64,
    triangles: u64,
    texture_bytes: u64,
) -> CachedTile {
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
    CachedTile {
        mesh,
        anchor,
        vertices,
        triangles,
        texture_bytes,
        material,
        images: image_handles,
    }
}

/// Executor seam for CPU tile builds. `TileKey` + shared field in,
/// [`TerrainBuildOutput`] out through the bounded channel; selection and
/// presentation never name the backend.
fn spawn_height_job(
    field: &Arc<PlanetField>,
    key: TileKey,
    tile_mesh_cells: usize,
    gpu_raster: bool,
) -> JobHandle<Result<HeightBuild<TerrainBuildOutput>, String>> {
    let field = field.clone();
    JobHandle::spawn(move || {
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
            let page = thessa_terrain_stream::height::bake_page(&field, key)?;
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
        Ok(HeightBuild {
            page: height_page,
            payload: TerrainBuildOutput {
                mesh,
                anchor,
                vertices,
                triangles,
                texture_bytes,
                images,
                seconds: [mesh_s, material_s],
            },
        })
    })
}

/// Presentation stage: advance ready page families independently of
/// planning-tree convergence. Every published partition still covers the
/// requested area; takes only the render snapshot borrows it needs.
fn publish_resident_cover(
    world: &mut WorldTerrain,
    topology: &mut CbtRenderTopology,
    pages: &CbtRenderPages,
) {
    if !world.cover_dirty || world.wanted.is_empty() {
        return;
    }
    world.cover_dirty = false;
    let previous: Vec<_> = world.visible.iter().copied().collect();
    if let Some(cover) =
        thessa_worldgen_rocky::streaming::resident_cover(&world.wanted, &previous, |key| {
            lod::cbt_node_for_tile(key).is_some_and(|node| pages.contains_page(node.id()))
        })
    {
        let cover: BTreeSet<_> = cover.into_iter().collect();
        if !cover.is_empty() && cover != world.visible {
            let nodes: Vec<_> = cover
                .iter()
                .filter_map(|key| lod::cbt_node_for_tile(*key))
                .collect();
            if topology.publish_resident_leaves(&nodes) {
                world.visible = cover;
            }
        }
    }
}

/// Presentation teardown for the inactive view mode. Hides the surface,
/// despawns tile entities and zeroes frame counters. Returns `true` when it
/// ran, in which case the caller has nothing else to do this frame.
#[allow(clippy::type_complexity)]
fn teardown_inactive_surface(
    commands: &mut Commands,
    world: &mut WorldTerrain,
    backdrop: &mut Query<
        (&mut Transform, &mut Visibility),
        (
            With<TerrainBackdrop>,
            Without<Camera3d>,
            Without<SurfaceTile>,
            Without<CelestialVisual>,
            Without<pilot::PilotPlanetVisual>,
        ),
    >,
    tiles: &mut Query<
        (Entity, &SurfaceTile, &mut Transform),
        (Without<Camera3d>, Without<TerrainBackdrop>),
    >,
    readout: &mut Query<&mut Text, With<SurveyReadout>>,
) -> bool {
    world.render_center = None;
    for (_, mut visibility) in backdrop {
        *visibility = Visibility::Hidden;
    }
    for (entity, _, _) in tiles {
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
    for mut text in readout {
        text.0.clear();
    }
    true
}

/// Frame view computed from survey/runtime state: eye, orientation, render
/// origin and selection frustum. Pure camera work — no tile selection, no
/// streaming; the returned view feeds the selection stage.
#[derive(Debug, Clone, Copy)]
struct FrameView {
    frame: BodyFrame,
    eye: DVec3,
    forward_body: DVec3,
    frustum: Option<lod::SelectionFrustum>,
    radius: f64,
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn update_frame_camera(
    world: &mut WorldTerrain,
    surface: &mut CbtRenderSurface,
    survey: &SurfaceSurvey,
    runtime: &PilotFlightRuntime,
    camera: &mut Single<
        (&mut Transform, &mut Projection),
        (With<Camera3d>, Without<TerrainBackdrop>),
    >,
    readout: &mut Query<&mut Text, With<SurveyReadout>>,
    backdrop: &mut Query<
        (&mut Transform, &mut Visibility),
        (
            With<TerrainBackdrop>,
            Without<Camera3d>,
            Without<SurfaceTile>,
            Without<CelestialVisual>,
            Without<pilot::PilotPlanetVisual>,
        ),
    >,
    gpu_raster: bool,
    gpu_cover_ready: bool,
) -> FrameView {
    let radius = world.field.params.radius_m;
    surface.set_radius_m(radius as f32);
    let (origin, rotation) = if survey.active {
        let (dir, label) = survey.sites[survey.site];
        let spin = DQuat::from_rotation_y(runtime.render_spin());
        let up = spin * DVec3::from_array(dir);
        let center = up * (radius + world.field.height_m(dir, 32.0).max(0.0));
        // Shared owned survey offsets; a degenerate site frame keeps the
        // previous camera instead of publishing NaNs.
        if let Some((offset, site_up)) = survey.camera.offsets(up) {
            let globe_eye = center + offset.as_dvec3();
            let minimum = radius
                + world
                    .field
                    .height_m((spin.inverse() * globe_eye.normalize()).to_array(), 32.0)
                    .max(0.0)
                + thessa_flight_authority::survey::MIN_ALTITUDE_MARGIN_M;
            let globe_eye = thessa_flight_authority::survey::SurveyCamera::enforce_minimum_altitude(
                globe_eye, minimum,
            );
            let offset = (globe_eye - center).as_vec3();
            camera.0.translation = offset;
            *camera.0 = camera.0.looking_at(Vec3::ZERO, site_up);
            camera.0.rotation *= survey.camera.look();
        }
        if let Projection::Perspective(p) = &mut *camera.1 {
            p.near = 1.0;
            p.far = 12_000_000.0;
        }
        // Sky and ambient stay owned by the atmosphere plugin: hardcoded
        // replacements here would fight the LUT sky and night side.
        for mut text in readout {
            text.0 = format!(
                "THESSA  /  {label}     {:.1} km",
                survey.camera.distance_m() / 1000.0
            );
        }
        (center, spin)
    } else {
        for mut text in readout {
            text.0.clear();
        }
        let origin = runtime.render_terrain_origin_m();
        let spin = DQuat::from_rotation_y(runtime.render_spin());
        (origin, spin)
    };
    let frame =
        BodyFrame::new(DVec3::ZERO, rotation, origin).expect("finite interpolated terrain pose");
    surface.set_render_from_body_f64(frame.render_from_body().to_cols_array());
    for (mut transform, mut visibility) in backdrop {
        transform.translation = frame.body_to_render_point(DVec3::ZERO).as_vec3();
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
        let globe_eye = frame.render_to_body_point(camera.0.translation.as_dvec3());
        let minimum = radius
            + world
                .field
                .height_m(globe_eye.normalize().to_array(), 32.0)
                .max(0.0)
            + 3.0;
        if globe_eye.length() < minimum {
            camera.0.translation = frame
                .body_to_render_point(globe_eye.normalize() * minimum)
                .as_vec3();
            let up = camera.0.up();
            *camera.0 = camera.0.looking_at(Vec3::ZERO, up);
        }
    }
    world.render_center = Some((-origin).as_vec3());
    world.render_origin_m = origin;
    frame_view(frame, &camera.0, &camera.1, radius)
}

/// Read the final host pose only after clearance/rebase edits. Selection and
/// CPU/GPU placement share one body frame rather than keeping a pre-clamp eye.
fn frame_view(
    frame: BodyFrame,
    camera: &Transform,
    projection: &Projection,
    radius: f64,
) -> FrameView {
    let eye = frame.render_to_body_point(camera.translation.as_dvec3());
    let forward_body = frame.render_to_body_direction((camera.rotation * Vec3::NEG_Z).as_dvec3());
    // View frustum in the body-fixed field frame, for selection culling:
    // from 5 km the horizon cone admits a 500 km disc that would eat any
    // tile budget before the nadir refines. Non-perspective projections
    // skip culling (full cover, no holes).
    let frustum = if let Projection::Perspective(p) = projection {
        let half_v = p.fov * 0.5;
        let half_h = (half_v.tan() * p.aspect_ratio).atan();
        // Wide swing margin: the cone must cover the frame CORNERS, not the
        // axes. A tight cone culls the ground below a forward-looking chase
        // camera (nadir is ~90° off-axis) and the pilot view never refines
        // past the coarse cover. Behind-camera tiles are still culled.
        let half_angle = f64::from(half_v.max(half_h)) + 0.6;
        Some(lod::SelectionFrustum {
            forward: forward_body.to_array(),
            cos_limit: half_angle.cos().clamp(-1.0, 1.0),
        })
    } else {
        None
    };
    FrameView {
        frame,
        eye,
        forward_body,
        frustum,
        radius,
    }
}

/// Selection stage: movement trigger -> tile demand -> CBT planning input.
///
/// Reads the frame view and the warm cache, writes `world.wanted` and
/// submits planning candidates. Returns `(selection_changed, detail_bias)`.
/// Takes only the planning borrows it needs; streaming and presentation
/// never observe its internals.
#[allow(clippy::too_many_arguments)]
fn run_tile_selection(
    world: &mut WorldTerrain,
    topology: &Tree,
    input: &mut CbtFrameInput,
    surface: &mut CbtRenderSurface,
    camera_projection: &Projection,
    perf: &mut perf::PerfMonitor,
    now_s: f64,
    gpu_raster: bool,
    view: &FrameView,
) -> (bool, f64) {
    let FrameView {
        frame,
        eye,
        radius,
        forward_body,
        frustum,
    } = *view;
    let selection_started = Instant::now();
    let selected = world.selection_history.update(
        &world.field,
        SelectionView {
            eye,
            stream_origin: frame.render_to_body_point(DVec3::ZERO),
            forward: forward_body,
            radius,
            frustum,
        },
        now_s,
        &world.fine_selection,
        world.wanted.is_empty(),
        gpu_raster,
        SelectionPolicy::default(),
    );
    let selection_changed = selected.is_some();
    let detail_bias = world.selection_history.detail_bias();
    if let Some(demand) = selected {
        world.fine_selection = demand.fine;
        world.wanted = demand.wanted;
        world.counters.terrain_cache_hits += world
            .wanted
            .iter()
            .filter(|key| world.heights.contains_key(key))
            .count() as u64;
        world.selection_epoch = world.selection_epoch.saturating_add(1);
        world.cover_dirty = true;
        world.topology_pending = true;
    }
    perf.record_scope(
        "world.terrain_selection",
        selection_started.elapsed().as_secs_f64(),
    );
    let fov_rad = match camera_projection {
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
    surface.set_view_state(
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
        let candidates = cbt_candidates_for_tiles(topology, &world.wanted);
        world.topology_pending = !candidates.is_empty();
        if world.topology_pending {
            input.submit(Some(render_view), candidates);
        }
    }
    perf.record_scope(
        "world.terrain_cbt_submit",
        cbt_submit_started.elapsed().as_secs_f64(),
    );
    (selection_changed, detail_bias)
}

/// Streaming stage: poll finished builds and convert host payloads. The height
/// snapshot is published once after admission/retirement, not between batches.
#[allow(clippy::too_many_arguments)]
fn poll_finished_pages(
    world: &mut WorldTerrain,
    meshes: &mut Assets<Mesh>,
    images: &mut Assets<Image>,
    materials: &mut Assets<StandardMaterial>,
    perf: &mut perf::PerfMonitor,
) {
    let poll_started = Instant::now();
    let poll = world.heights.poll_with(world.selection_epoch, |_, output| {
        let TerrainBuildOutput {
            mesh: built_mesh,
            anchor,
            vertices,
            triangles,
            texture_bytes,
            images: built_images,
            seconds,
        } = output;
        perf.record_scope("world.terrain_meshing", seconds[0]);
        perf.record_scope("world.terrain_materials", seconds[1]);
        present_cpu_tile(
            meshes,
            images,
            materials,
            built_mesh,
            built_images,
            anchor,
            vertices,
            triangles,
            texture_bytes,
        )
    });
    for (key, error) in &poll.failed {
        eprintln!("[terrain] height job for {key:?}: {error}; retrying demand");
    }
    release_height_assets(poll.retired, meshes, images, materials);
    world.counters.terrain_patches_generated += poll.ready as u32;
    world.cover_dirty |= poll.ready > 0;
    perf.record_scope(
        "world.terrain_job_poll_and_asset_upload",
        poll_started.elapsed().as_secs_f64(),
    );
}

/// Requests stage: order the demand cover and spawn height builds up to the
/// in-flight and cache budgets. Dependencies/LRU/admission belong to the shared
/// lifecycle; spawning goes through the [`spawn_height_job`] executor seam.
fn spawn_height_jobs(
    world: &mut WorldTerrain,
    terrain_mesh_cells: usize,
    gpu_raster: bool,
    view: &FrameView,
) -> Vec<(TileKey, CachedTile)> {
    let field = world.field.clone();
    let admission = world.heights.reconcile(
        &field,
        SelectionView {
            eye: view.eye,
            radius: view.radius,
            forward: view.forward_body,
            stream_origin: DVec3::ZERO,
            frustum: view.frustum,
        },
        &world.wanted,
        world.visible.iter().copied(),
        world.selection_epoch,
        HeightPolicy {
            capacity: if gpu_raster { 1024 } else { 512 },
            max_jobs: 8,
            ancestors: gpu_raster,
            pin_roots: false,
        },
        |key| {
            let tile_mesh_cells = if gpu_raster {
                32
            } else {
                mesh_cells_for_tile(terrain_mesh_cells, key)
            };
            spawn_height_job(&field, key, tile_mesh_cells, gpu_raster)
        },
    );
    world.counters.terrain_cache_misses += admission.spawned as u64;
    if admission.capacity_rejected && !world.height_capacity_rejected {
        warn!(
            "terrain height cache capacity exhausted: {} residents + {} pending; retaining current cover",
            world.heights.len(),
            world.heights.pending_count()
        );
    }
    world.height_capacity_rejected = admission.capacity_rejected;
    admission.retired
}

/// Refine the existing view selection for material texel coverage, not mesh
/// curvature. A 128px colour page can require more refinement than its mesh.
#[cfg(test)]
fn material_view_selection(base: &[TileKey], view: &FrameView, pixel_angle: f64) -> Vec<TileKey> {
    selection::material_view_selection(
        base,
        SelectionView {
            eye: view.eye,
            stream_origin: DVec3::ZERO,
            forward: view.forward_body,
            radius: view.radius,
            frustum: view.frustum,
        },
        pixel_angle,
    )
}

/// Fill a useful regional page before spending a full 128² build on each
/// close tile. Four levels cover up to 256 descendants; subsequent steps
/// refine by two levels. This changes residency order, never field accuracy.
#[cfg(test)]
fn material_build_dependency(key: TileKey, mut resident: impl FnMut(TileKey) -> bool) -> TileKey {
    selection::material_build_dependency(key, &mut resident)
}

/// Streaming stage for material pages (GPU path only): poll finished pages,
/// refresh the wanted set through the shared material lifecycle. Height/cover jobs never wait for
/// it; a resident canonical globe map remains the fallback.
fn run_material_streaming(
    world: &mut WorldTerrain,
    material_pages: &mut ResMut<'_, thessa_bevy_rcbt::CbtRenderMaterialPages>,
    gpu_raster: bool,
    view: &FrameView,
    pixel_angle: f64,
) {
    if !gpu_raster {
        return;
    }
    let poll = world.material_stream.poll();
    if poll.failed > 0 {
        eprintln!(
            "[terrain] {} material worker(s) disconnected; retrying demand",
            poll.failed
        );
    }
    world.material_stream.reconcile(
        &world.field,
        MaterialDemand {
            selection_epoch: world.selection_epoch,
            fine: &world.fine_selection,
            view: SelectionView {
                eye: view.eye,
                radius: view.radius,
                forward: view.forward_body,
                stream_origin: DVec3::ZERO,
                frustum: view.frustum,
            },
            pixel_angle,
            height_jobs: world.heights.pending_count(),
        },
        world.visible.iter().copied().chain(
            world
                .wanted
                .iter()
                .copied()
                .filter(|key| world.heights.contains_key(key)),
        ),
    );
    // Shared borrow first: taking &mut through ResMut marks Bevy changed even
    // for identical bytes. Only dirty revisions cross this publication bridge.
    thessa_bevy_rcbt::CbtRenderMaterialPages::publish_if_changed(
        world.material_stream.source(),
        material_pages,
    );
}

/// CPU-fallback presentation bridge — the only stage that spawns Bevy tile
/// entities. Syncs the mesh-path entity cover, the bootstrap backdrop, tile
/// transforms and celestial visibility. The GPU path never creates entities
/// here: `world.visible` is the retained complete draw snapshot.
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn sync_tile_entities(
    commands: &mut Commands,
    world: &mut WorldTerrain,
    tiles: &mut Query<
        (Entity, &SurfaceTile, &mut Transform),
        (Without<Camera3d>, Without<TerrainBackdrop>),
    >,
    backdrop: &mut Query<
        (&mut Transform, &mut Visibility),
        (
            With<TerrainBackdrop>,
            Without<Camera3d>,
            Without<SurfaceTile>,
            Without<CelestialVisual>,
            Without<pilot::PilotPlanetVisual>,
        ),
    >,
    celestial: &mut Query<
        &mut Visibility,
        Or<(With<CelestialVisual>, With<pilot::PilotPlanetVisual>)>,
    >,
    sky: Option<&water::WaterSky>,
    surface_raster_enabled: bool,
    gpu_raster: bool,
    gpu_cover_ready: bool,
    selection_changed: bool,
    view: &FrameView,
    perf: &mut perf::PerfMonitor,
) {
    let frame = view.frame;
    let entity_sync_started = Instant::now();
    if !gpu_raster && (selection_changed || world.counters.terrain_patches_generated > 0) {
        let desired = ready_terrain_cover(&world.wanted, &world.visible, |key| {
            world.heights.contains_key(key)
        });
        if desired != world.visible {
            for (entity, tile, _) in &mut *tiles {
                if !desired.contains(&tile.0) {
                    commands.entity(entity).despawn();
                }
            }
            for key in desired.difference(&world.visible) {
                let tile = world.heights.get(key).expect("ready tile");
                if surface_raster_enabled {
                    continue;
                }
                let (Some(mesh), Some(material)) = (&tile.mesh, &tile.material) else {
                    continue;
                };
                let tile_entity = commands.spawn((
                    SurfaceTile(*key),
                    Mesh3d(mesh.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_translation(frame.body_to_render_point(tile.anchor).as_vec3())
                        .with_rotation(frame.rotation().as_quat()),
                    Name::new(format!("Thessa tile {key:?}")),
                ));
                // Shared sky probe at spawn (atomic with creation: no
                // query/insert race when cover churns under warp).
                if let Some(sky) = sky {
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
    for (_, mut visibility) in backdrop {
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
    for (_, tile, mut transform) in &mut *tiles {
        if let Some(cached) = world.heights.get(&tile.0) {
            transform.translation = frame.body_to_render_point(cached.anchor).as_vec3();
            transform.rotation = frame.rotation().as_quat();
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
        for mut visibility in celestial {
            *visibility = Visibility::Hidden;
        }
    }
}

/// Host-only retirement bridge. The shared height lifecycle returns each
/// evicted/replaced payload exactly once; asset ownership never enters its API.
fn release_height_assets(
    retired: impl IntoIterator<Item = (TileKey, CachedTile)>,
    meshes: &mut Assets<Mesh>,
    images: &mut Assets<Image>,
    materials: &mut Assets<StandardMaterial>,
) {
    for (_, tile) in retired {
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

/// User-visible material holes: visible-cover tiles with no resident
/// material page at any level (exact or ancestor), so their fragments fall
/// back to the globe map. This deliberately ignores the 512-entry bounded
/// streaming queue: under pressure the queue truncates demand and a
/// priority∩¬resident count reads low while the screen still shows globe
/// fallback. No autobench gate consumes this counter (verified: only
/// telemetry plumbed through `WorldCounters`); it is a holes gauge, not a
/// queue-saturation gauge.
fn missing_material_holes(
    visible: &BTreeSet<TileKey>,
    pages: &thessa_bevy_rcbt::CbtRenderMaterialPages,
) -> u32 {
    visible
        .iter()
        .filter(|key| {
            let Some(node) = lod::cbt_node_for_tile(**key) else {
                return false;
            };
            thessa_bevy_rcbt::material_cache::resolve_material_ancestor(node.id(), |id| {
                pages.contains_page(id)
            })
            .is_none()
        })
        .count() as u32
}

/// Telemetry stage: roll cache/streaming/selection state into counters.
/// Reads only (plus the counters themselves); runs last so every stage's
/// work is reflected in the same frame's numbers.
fn record_terrain_counters(
    world: &mut WorldTerrain,
    material_pages: &thessa_bevy_rcbt::CbtRenderMaterialPages,
    detail_bias: f64,
    perf: &mut perf::PerfMonitor,
    started: Instant,
) {
    let counters_started = Instant::now();
    world.counters.terrain_cache_entries = world.heights.len() as u32;
    world.counters.terrain_material_jobs = world.material_stream.pending_count() as u32;
    world.counters.terrain_selection_changes = world.selection_epoch;
    world.counters.terrain_material_missing =
        missing_material_holes(&world.visible, material_pages);
    world.counters.assets_loaded = world.heights.len() as u32 * 5;
    world.counters.assets_pending = world.heights.pending_count() as u32 * 5;
    world.counters.terrain_patches_visible = world.visible.len() as u32;
    world.counters.terrain_patches_wanted = world.wanted.len() as u32;
    world.counters.terrain_jobs_in_flight = world.heights.pending_count() as u32;
    world.counters.terrain_eye_speed_mps =
        world.selection_history.eye_speed_mps.round().max(0.0) as u32;
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
        .filter(|k| !world.heights.contains_key(k))
        .count() as u32;
    world.counters.terrain_vertices = world
        .visible
        .iter()
        .filter_map(|k| world.heights.get(k))
        .map(|t| t.vertices)
        .sum();
    world.counters.terrain_triangles = world
        .visible
        .iter()
        .filter_map(|k| world.heights.get(k))
        .map(|t| t.triangles)
        .sum();
    world.cache_bytes = world
        .heights
        .values()
        .map(|t| t.vertices * 48 + t.triangles * 12 + t.texture_bytes)
        .sum::<u64>()
        + material_pages.byte_len() as u64;
    perf.record_scope(
        "world.terrain_counters",
        counters_started.elapsed().as_secs_f64(),
    );
    perf.record_scope("world.streaming", started.elapsed().as_secs_f64());
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
        Query<&Window, With<bevy::window::PrimaryWindow>>,
    ),
    sky: Option<Res<water::WaterSky>>,
) {
    let (mut readout, mut backdrop, windows) = display;
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
    // The same coarse-to-fine cover remains valid in orbit. An altitude gate
    // swapped the whole surface to a different sphere at 80 km and bypassed
    // the presentation fence during physical ascents/descents.
    let active = survey.active || pilot.view_mode == ClientViewMode::Pilot;
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
    // Presentation advances ready families independently of planning-tree
    // convergence. Every published partition still covers the requested area.
    let stage_started = Instant::now();
    if active && gpu_raster && gpu_albedo_ready {
        publish_resident_cover(&mut world, &mut cbt.topology, &cbt.pages);
    }
    perf.record_scope(
        "world.terrain_publish",
        stage_started.elapsed().as_secs_f64(),
    );
    // Keep the last complete draw snapshot and its pages while workers and
    // the planning tree converge. Never flash the bootstrap on each LOD change.
    let cpu_cover_ready = gpu_raster && gpu_albedo_ready && !world.visible.is_empty();
    // Enable draw preparation first. Hide the bootstrap only after the render
    // world has actually bound the GPU images and submitted this surface.
    cbt.surface.set_gpu_surface_ready(active && cpu_cover_ready);
    let gpu_cover_ready = cbt
        .presentation
        .is_ready(&cbt.surface, cbt.material.as_deref());
    world.counters.terrain_patches_generated = 0;
    if !active {
        teardown_inactive_surface(
            &mut commands,
            &mut world,
            &mut backdrop,
            &mut tiles,
            &mut readout,
        );
        return;
    }
    let view = {
        let stage_started = Instant::now();
        let view = update_frame_camera(
            &mut world,
            &mut cbt.surface,
            &survey,
            &runtime,
            &mut camera,
            &mut readout,
            &mut backdrop,
            gpu_raster,
            gpu_cover_ready,
        );
        perf.record_scope(
            "world.terrain_frame_camera",
            stage_started.elapsed().as_secs_f64(),
        );
        view
    };
    let now_s = time.elapsed_secs_f64();
    // Selection stage: movement trigger -> tile demand -> CBT planning.
    let (selection_changed, detail_bias) = run_tile_selection(
        &mut world,
        cbt.state.topology(),
        &mut cbt.input,
        &mut cbt.surface,
        &camera.1,
        &mut perf,
        now_s,
        gpu_raster,
        &view,
    );
    // Streaming stage: poll finished builds and convert host asset payloads.
    let stage_started = Instant::now();
    poll_finished_pages(
        &mut world,
        &mut meshes,
        &mut images,
        &mut materials,
        &mut perf,
    );
    perf.record_scope("world.terrain_poll", stage_started.elapsed().as_secs_f64());
    // Shared requests/cache stage: pin coverage, retire cold payloads, admit
    // bounded work, then publish one source snapshot for the final state.
    let stage_started = Instant::now();
    let retired = spawn_height_jobs(&mut world, terrain_mesh_cells, gpu_raster, &view);
    world.counters.terrain_cache_evictions += retired.len() as u64;
    release_height_assets(retired, &mut meshes, &mut images, &mut materials);
    CbtRenderPages::publish_if_changed(world.heights.source(), &mut cbt.pages);
    perf.record_scope("world.terrain_spawn", stage_started.elapsed().as_secs_f64());
    // Material streaming stage (GPU path only).
    let pixel_angle = match &*camera.1 {
        Projection::Perspective(projection) => {
            let height = windows
                .iter()
                .next()
                .map_or(720.0, |window| f64::from(window.physical_height()));
            2.0 * f64::from(projection.fov * 0.5).tan() / height.max(1.0)
        }
        // Map/ortho views have no angular pixel: derive the equivalent from
        // the orthographic frustum height so they refine instead of sitting
        // on permanent coarse pages. `world_per_pixel / eye_dist` keeps the
        // `distance * pixel_angle` footprint math in `material_view_selection`
        // correct for nadir tiles; limb tiles only over-refine slightly.
        Projection::Orthographic(projection) => {
            let height = windows
                .iter()
                .next()
                .map_or(720.0, |window| f64::from(window.physical_height()));
            let frustum_h = (projection.area.max.y - projection.area.min.y) as f64;
            let world_per_pixel = frustum_h / height.max(1.0);
            (world_per_pixel / view.eye.length().max(1.0)).clamp(1e-6, 1.0)
        }
        _ => 1.0,
    };
    let stage_started = Instant::now();
    run_material_streaming(
        &mut world,
        &mut cbt.material_pages,
        gpu_raster,
        &view,
        pixel_angle,
    );
    perf.record_scope(
        "world.terrain_material_stream",
        stage_started.elapsed().as_secs_f64(),
    );
    // CPU-fallback presentation bridge: entities, backdrop, transforms.
    sync_tile_entities(
        &mut commands,
        &mut world,
        &mut tiles,
        &mut backdrop,
        &mut celestial,
        sky.as_deref(),
        cbt.surface.gpu_raster_enabled(),
        gpu_raster,
        gpu_cover_ready,
        selection_changed,
        &view,
        &mut perf,
    );
    // Telemetry stage: counters for this frame.
    record_terrain_counters(
        &mut world,
        &cbt.material_pages,
        detail_bias,
        &mut perf,
        started,
    );
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
    fn material_missing_counts_visible_holes_not_queue_saturation() {
        use thessa_worldgen_rocky::lod::TileKey;
        // Two visible tiles: one with an exact resident page, one with only
        // a resident ancestor (covered, not a hole), plus one with nothing.
        let visible = BTreeSet::from([
            TileKey {
                face: 0,
                level: 2,
                x: 0,
                y: 0,
            },
            TileKey {
                face: 0,
                level: 2,
                x: 1,
                y: 0,
            },
            TileKey {
                face: 1,
                level: 1,
                x: 0,
                y: 0,
            },
        ]);
        let mut pages = thessa_bevy_rcbt::CbtRenderMaterialPages::default();
        assert_eq!(missing_material_holes(&visible, &pages), 3);
        // Exact page for the first tile.
        let page = thessa_bevy_rcbt::CbtMaterialPage::from_decoded_mips(vec![vec![
            128;
            (lod::GPU_MATERIAL_PAGE_SIZE.pow(2) * 4)
                as usize
        ]])
        .unwrap();
        let first = lod::cbt_node_for_tile(TileKey {
            face: 0,
            level: 2,
            x: 0,
            y: 0,
        })
        .unwrap();
        pages.set_pages([(first.id(), page.clone())]);
        assert_eq!(missing_material_holes(&visible, &pages), 2);
        // Resident ancestor covers the second tile without an exact page:
        // not a user-visible hole (globe fallback is not used).
        let parent = lod::cbt_node_for_tile(TileKey {
            face: 0,
            level: 1,
            x: 0,
            y: 0,
        })
        .unwrap();
        pages.set_pages([(parent.id(), page)]);
        assert_eq!(missing_material_holes(&visible, &pages), 1);
        // Queue truncation must not move the holes gauge: priority may hold
        // hundreds of pending ids while the visible cover is unaffected.
        pages.set_priority(vec![8, 9]);
        assert_eq!(missing_material_holes(&visible, &pages), 1);
    }

    #[test]
    fn material_bootstrap_covers_regions_then_refines_to_exact_demand() {
        let wanted = TileKey {
            face: 4,
            level: 14,
            x: 1024,
            y: 4096,
        };
        let initial = material_build_dependency(wanted, |_| false);
        assert_eq!(initial.level, 10);
        let mut resident = BTreeSet::from([initial]);
        for level in [12, 14] {
            let next = material_build_dependency(wanted, |key| resident.contains(&key));
            assert_eq!(next.level, level);
            resident.insert(next);
        }
        assert_eq!(
            material_build_dependency(wanted, |key| resident.contains(&key)),
            wanted
        );
        for face in 0..6 {
            let root = TileKey::root(face);
            assert_eq!(material_build_dependency(root, |_| false), root);
            assert_eq!(
                material_build_dependency(root.children()[0], |_| false),
                root
            );
        }
    }

    #[test]
    fn terrain_frame_reads_final_rebased_camera_after_clearance_edits() {
        let radius = 3_200_000.0;
        let spin = DQuat::from_rotation_y(0.37);
        let origin = spin * (DVec3::Z * radius);
        let frame = BodyFrame::new(DVec3::ZERO, spin, origin).unwrap();
        let mut camera = Transform::from_translation(Vec3::new(0.0, 0.0, -10.0));
        let projection = Projection::Perspective(PerspectiveProjection::default());
        let before = frame_view(frame, &camera, &projection, radius);
        // The clearance stage changes host-local translation before selection.
        let clamped = DVec3::Z * (radius + 3.0);
        camera.translation = frame.body_to_render_point(clamped).as_vec3();
        camera.rotation = spin.as_quat();
        let after = frame_view(frame, &camera, &projection, radius);
        assert!((after.eye - clamped).length() < 1e-6);
        assert!((before.eye - after.eye).length() > 5.0);
        assert!((after.forward_body - DVec3::NEG_Z).length() < 1e-7);
        assert_eq!(
            after.frustum.unwrap().forward,
            after.forward_body.to_array()
        );
        let from_matrix = frame.render_from_body().transform_point3(clamped);
        assert!((from_matrix - frame.body_to_render_point(clamped)).length() < 1e-8);
    }

    #[test]
    fn material_demand_refines_coarse_geometry_without_changing_its_cover() {
        let base = [TileKey {
            face: 0,
            level: 8,
            x: 128,
            y: 128,
        }];
        let radius = 3_200_000.;
        let view = FrameView {
            frame: BodyFrame::new(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO).unwrap(),
            eye: DVec3::from_array(base[0].direction(0.5, 0.5)) * (radius + 2000.),
            forward_body: DVec3::NEG_X,
            frustum: None,
            radius,
        };
        let pages = material_view_selection(&base, &view, 0.001);
        assert!(pages.len() <= 512);
        assert!(pages.iter().all(|key| key.level > base[0].level));
        let area: f64 = pages
            .iter()
            .map(|key| 4.0_f64.powi(-i32::from(key.level - base[0].level)))
            .sum();
        assert!((area - 1.).abs() < 1e-10);
        assert_eq!(
            base[0].level, 8,
            "material demand must not refine geometry authority"
        );
    }

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

    fn overcast_image() -> Image {
        use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
        Image::new(
            Extent3d {
                width: 8,
                height: 4,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            vec![200u8; 8 * 4 * 4],
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::RENDER_WORLD,
        )
    }

    #[test]
    fn height_payload_replacement_releases_only_retired_bevy_assets() {
        let mut meshes = Assets::<Mesh>::default();
        let mut images = Assets::<Image>::default();
        let mut materials = Assets::<StandardMaterial>::default();
        let mut heights = HeightStream::<CachedTile, ()>::default();
        let key = TileKey::root(4);
        for height in [10.0, 20.0] {
            let batch = heights.insert_batch(
                0,
                [(
                    key,
                    HeightBuild {
                        page: thessa_rcbt_core::HeightPage::bake(&[height; 4], 2, 0.01).unwrap(),
                        payload: (),
                    },
                )],
                |_, ()| {
                    present_cpu_tile(
                        &mut meshes,
                        &mut images,
                        &mut materials,
                        Some(Mesh::from(Cuboid::default())),
                        Some([overcast_image(), overcast_image(), overcast_image()]),
                        DVec3::ZERO,
                        24,
                        12,
                        768,
                    )
                },
            );
            assert_eq!(batch.ready, 1);
            release_height_assets(batch.retired, &mut meshes, &mut images, &mut materials);
            assert_eq!((meshes.len(), materials.len(), images.len()), (1, 1, 3));
            let tile = heights.get(&key).unwrap();
            assert!(meshes.get(tile.mesh.as_ref().unwrap()).is_some());
            assert!(materials.get(tile.material.as_ref().unwrap()).is_some());
            for handle in tile.images.as_ref().unwrap() {
                assert!(images.get(handle).is_some());
            }
        }
        // GPU payload presentation creates no hidden fallback assets.
        let empty = present_cpu_tile(
            &mut meshes,
            &mut images,
            &mut materials,
            None,
            None,
            DVec3::ZERO,
            0,
            0,
            0,
        );
        release_height_assets([(key, empty)], &mut meshes, &mut images, &mut materials);
        assert_eq!((meshes.len(), materials.len(), images.len()), (1, 1, 3));
    }

    /// Minimal world for the effect systems: visible planet, sun, overcast
    /// deck, one CPU tile. No terrain field needed (the entity match wins).
    fn effect_world(shadows: bool, cities: bool) -> World {
        let mut world = World::new();
        world.insert_resource(CbtRenderMaterial::default());
        world.insert_resource(GraphicsResolved(ResolvedGraphicsSettings {
            clouds_cast_shadows: shadows,
            night_lights_enabled: cities,
            night_lights_intensity: 1.0,
            ..Default::default()
        }));
        world.insert_resource(SimulationClock::default());
        world.insert_resource(Time::<()>::default());
        let mut images = Assets::<Image>::default();
        let cloud = images.add(overcast_image());
        world.insert_resource(beauty::BeautyImages::test_with_clouds(cloud));
        world.insert_resource(images);
        world.insert_resource(Assets::<StandardMaterial>::default());
        world.spawn((
            Name::new("thessa"),
            Transform {
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                scale: Vec3::splat(3_200_000.0),
            },
            InheritedVisibility::VISIBLE,
        ));
        world.spawn((GlobalTransform::IDENTITY, PrimaryStarLight));
        world
    }

    #[test]
    fn effect_sync_publishes_shadow_and_city_samplers() {
        let mut world = effect_world(true, true);
        let city = {
            let mut lights = world.resource_mut::<Assets<Image>>();
            lights.add(overcast_image())
        };
        world.insert_resource(crate::night_lights::NightLights { image: city });
        let mut schedule = Schedule::default();
        schedule.add_systems(sync_cbt_effects);
        schedule.run(&mut world);
        let material = world.resource::<CbtRenderMaterial>();
        let shadow = material.cloud_shadow.as_ref().expect("shadow sampler");
        // Unit rotation and sun direction (valid shader uniforms).
        let qlen = (shadow.relative_rotation.iter().map(|v| v * v).sum::<f32>()).sqrt();
        assert!((qlen - 1.0).abs() < 1e-5, "{qlen}");
        let slen = (shadow.sun_body_dir[0].powi(2)
            + shadow.sun_body_dir[1].powi(2)
            + shadow.sun_body_dir[2].powi(2))
        .sqrt();
        assert!((slen - 1.0).abs() < 1e-5, "{slen}");
        assert_eq!(
            shadow.deck_height_m,
            beauty::CLOUD_SHADOW_DECK_HEIGHT_M as f32
        );
        assert!(material.night_lights.is_some());
    }

    #[test]
    fn effect_sync_clears_samplers_when_disabled() {
        let mut world = effect_world(false, false);
        let mut schedule = Schedule::default();
        schedule.add_systems(sync_cbt_effects);
        schedule.run(&mut world);
        let material = world.resource::<CbtRenderMaterial>();
        assert!(material.cloud_shadow.is_none());
        assert!(material.night_lights.is_none());
    }

    #[test]
    fn tile_shadows_darken_under_overcast_and_hold_when_off() {
        for enabled in [true, false] {
            let mut world = effect_world(enabled, false);
            let mat = {
                let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
                materials.add(StandardMaterial {
                    base_color: Color::WHITE,
                    ..Default::default()
                })
            };
            world.spawn((
                SurfaceTile(TileKey {
                    face: 0,
                    level: 10,
                    x: 0,
                    y: 0,
                }),
                GlobalTransform::from_translation(Vec3::new(3_200_000.0, 0.0, 0.0)),
                MeshMaterial3d(mat.clone()),
            ));
            world
                .resource_mut::<Time>()
                .advance_by(std::time::Duration::from_secs_f32(0.5));
            let mut schedule = Schedule::default();
            schedule.add_systems(update_tile_cloud_shadows);
            schedule.run(&mut world);
            let materials = world.resource::<Assets<StandardMaterial>>();
            let got = materials.get(&mat).unwrap().base_color.to_linear();
            if enabled {
                // Overcast alpha 200/255 must land exactly on the analytic
                // shadow factor (through sRGB, as the material stores it).
                let f = beauty::shadow_factor_from_alpha(200.0 / 255.0);
                let expected = Color::srgb(f, f, f).to_linear().red;
                assert!(
                    (got.red - expected).abs() < 0.02,
                    "shadowed tile {got:?} vs {expected}"
                );
            } else {
                assert_eq!(got, LinearRgba::WHITE);
            }
        }
    }

    #[test]
    fn tile_shadows_hold_when_gpu_draws_terrain() {
        // GPU raster shades the deck per-pixel in-shader; the CPU per-tile
        // path must not touch materials there (its constants print
        // tile-edged quads over the smooth field, screenshot-caught).
        let mut world = effect_world(true, false);
        world.resource_mut::<GraphicsResolved>().0.terrain =
            thessa_graphics::ResolvedTerrainRender::GpuIndexed;
        let mat = {
            let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
            materials.add(StandardMaterial {
                base_color: Color::WHITE,
                ..Default::default()
            })
        };
        world.spawn((
            SurfaceTile(TileKey {
                face: 0,
                level: 10,
                x: 0,
                y: 0,
            }),
            GlobalTransform::from_translation(Vec3::new(3_200_000.0, 0.0, 0.0)),
            MeshMaterial3d(mat.clone()),
        ));
        world
            .resource_mut::<Time>()
            .advance_by(std::time::Duration::from_secs_f32(0.5));
        let mut schedule = Schedule::default();
        schedule.add_systems(update_tile_cloud_shadows);
        schedule.run(&mut world);
        let materials = world.resource::<Assets<StandardMaterial>>();
        let got = materials.get(&mat).unwrap().base_color.to_linear();
        assert_eq!(got, LinearRgba::WHITE);
    }
}
