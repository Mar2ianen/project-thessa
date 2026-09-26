# Cabin editor

Status: slices 5–7 implemented (suits, venting/EVA rules, control
authority); slices 1–4 (seat blocks, classes/monuments, doors, decks)
remain design before implementation. Capsule cabins already exist as a
compiled primitive (`crates/fuselage/src/capsule.rs`); this note designs
the common cabin layer that will also serve airliners, supersonic
transports, and fighter cockpits.

## 1. Goal

One cabin authoring model must be able to produce, from the same
primitives:

- a 747-class double-deck airliner cabin (twin-aisle, several classes,
  galleys, lavatories, many doors);
- a Concorde-class slender supersonic cabin (narrow single-aisle,
  ~100 places, tight width budget);
- a fighter cockpit (single or tandem ejection seats, canopy,
  instrument mass, pressure suits);
- the existing capsule couches (they stay as they are and become one
  preset family of this model).

Non-goals for the cabin layer: evacuation *simulation* (the 90-second
rule stays an operational check, §7), catering logistics, ticket
pricing, cabin lighting/IFE cosmetics, and active ECLSS loops (oxygen
bookkeeping already exists as air/O2 mass; metabolic consumption is a
later slice).

## 2. What varies between the families

| Aspect | 747-class | Concorde-class | Fighter | Capsule (exists) |
|---|---|---|---|---|
| Aisles | 2 (twin-aisle) | 1 | 0 (tandem) / n/a | 0 |
| Decks | 2 (main + upper) | 1 | 1 | 1 |
| Seat style | upright, several classes | upright, 1–2 classes | ejection seat | couch |
| Width driver | 10-abreast + aisles | 4-abreast + aisle | 1-abreast + rails | 1–3 abreast |
| Length driver | hundreds of rows | ~25 rows | 1–2 places | 1–4 couches |
| Doors | many Type-A pairs | few small doors | canopy (cutout) | hatch + dock |
| Crew beyond pilots | dozens of attendants | handful | none / WSO | none |
| Pressure setpoint | ~75 kPa cabin alt | higher Δp setpoint | low-pressure + mask | sea-level |
| Suits | rarely (ferry) | rarely | pressure suit, always | worn launch/entry |
| Control source | pilots + autopilot | pilots + autopilot | pilot (or core) | pilot / core / none |

Everything in the table is a parameter of one model, not a separate
physics class — the same invariant as fuselages (`03`, §1).

## 3. Unified model

A cabin lives inside one `InteriorRegion` axial range and reuses its
atmosphere, mass aggregation, and seat-anchor pipeline. New entities:

```text
CabinLayout {
    decks: Vec<Deck>,          // usually one; 747 has two
}

Deck {
    floor_z_m: f64,            // deck height in body metres
    min_headroom_m: f64,       // 2.0 airliner, lower for fighters/capsules
    blocks: Vec<SeatBlock>,    // axially packed, no overlaps
    monuments: Vec<Monument>,  // galleys, lavs, closets
    doors: Vec<Door>,          // per side
}

SeatBlock {
    class: SeatClass,          // economy / premium / business / first / ejection
    columns: Vec<u32>,         // seat groups split by aisles, e.g. [3, 4, 3]
    rows: u32,
    pitch_m: f64,
    seat_mass_kg_each: f64,    // authored; class presets only suggest
    occupant_mass_kg_each: f64,
    carry_on_kg_each: f64,
    suited: bool,              // pressure suit: cabin air optional (§7)
    suit_mass_kg_each: f64,    // authored; suit presets only suggest
    suit_type: SuitType,       // hose-fed vs self-contained (§7–8)
}

Monument {
    kind: Galley | Lavatory | Closet | FlightDeck | AvionicsRack,
    x0_m, x1_m: f64,           // footprint along the deck
    mass_kg: f64,              // authored fitted mass
}

Door {
    x_m: f64,
    side: Left | Right,
    rating: ExitType,          // A / B / C / I / II / III
}
```

`SeatBlock.columns` generalizes the existing `abreast` (a `[3]` block
is today's 3-abreast row; `[3, 4, 3]` is a 747 row). Aisles between
column groups get an authored width each (typical 0.51 m twin-aisle,
0.43 m single-aisle — authored, not magic).

`SeatClass` is a preset bundle (width, suggested mass), never a hidden
multiplier: economy ≈ 0.44 m / ~11 kg, premium ≈ 0.47 m / ~18 kg,
business lie-flat ≈ 0.55 m / ~60 kg, first suite ≈ 0.65 m / ~100 kg,
ejection ≈ 0.55 m / ~110 kg with rails and kit. All illustrative
typical values; every number stays overridable per block, and the
compiler uses only the authored values.

`SuitType` is `HoseFed` (fighter pressure suit, capsule launch/entry
suit: ~15–25 kg, vehicle-fed air) vs `SelfContained` (EVA suit:
~100–130 kg with PLSS backpack, duration-limited consumables later).
Illustrative masses again; authored values rule. A suited block does
not require cabin pressure (§7); an unsuited block without atmosphere
fails closed at compile time (ground-ambient ferry ops stay a future
scenario flag, not a silent exception).

## 4. Geometry-derived validation (fails closed)

All checks derive from the loft at the actual row/door x-stations:

1. **Width fit, per row.** Sum of seat widths + aisle widths + wall
   clearance ≤ inner section width at `(x_row, floor_z_m)`. This is the
   rule that makes a 3-4-3 block fail inside a Concorde-width loft and
   pass inside a 747-width loft, with no per-vehicle tuning.
2. **Length packing.** Blocks (rows × pitch) + monument footprints +
   door clear zones fit the region length without overlaps — the same
   no-overlap discipline as interior regions.
3. **Headroom.** Section crown above `floor_z_m + min_headroom_m` at
   every seated row x. The 747 upper deck passes only inside the hump;
   a full-height block fails near the nose taper.
4. **Door fit.** Door height for its `ExitType` ≤ local section height
   at `x_m`; doors need shell on their side (always true on a loft,
   recorded for future cutouts).
5. **Exit-limited occupancy.** Sum of door ratings on board ≥ total
   occupants. Ratings come from a project-owned exit table keyed by
   `ExitType` (indicative magnitudes only at design time — Type A
   of order 100, overwing of order dozens; exact values lock against
   14 CFR / CS 25.807 with source cited at implementation).
6. **Crew complement.** 2 pilots minimum when the layout carries
   passengers (flight-deck monument or explicit crew), plus one cabin
   attendant place per 50 passenger seats or part thereof (FAR 121.391
   family; exact regulatory citation locks at implementation).
   Attendant seats are ordinary upright places in a crew block.

The 90-second evacuation demonstration stays out of the compiler: it
is an operational validation fed by door/aisle geometry, not a
hangar-time pass/fail.

## 5. Mass model

- Seats: `rows × places × seat_mass_kg_each` at anchor positions
  (existing anchor pipeline, extended with class tags later).
- Occupants + carry-on: same anchors, like today's seat/occupant mass.
- Monuments: fitted mass at footprint centroid.
- Checked baggage: not cabin mass — it goes to `Cargo` manifest in the
  hold, which already exists.
- Flight deck: pilots as occupants of a 2-place block; instrument mass
  as an `Avionics`-style manifest entry (fighters) or monument mass.
- Ejection seats: authored mass includes rails/kit; the seat stays with
  the airframe in the mass budget (ejection event dynamics are future).

## 6. Pressure reuse

Cabins reuse the per-region atmosphere (pressure/temperature/O2) and
the skin pressure screening unchanged. Only the setpoints differ:
airliner ~75 kPa equivalent, Concorde higher-Δp schedule, fighter
low-pressure plus mask (mask/O2-system detail is future ECLSS), capsule
sea-level. The existing `CABIN_ALTITUDE` alert contract already keys
off pressurized occupied volumes. Venting a cabin to vacuum (§8) is a
runtime state change of the same atmosphere record, not a rebuild.

## 7. Suits and unpressurized operations

A suited occupant brings their own pressure, so the cabin does not have
to. Compile rule: a block with `suited = false` and no atmosphere
refuses (add air or suits); a suited block compiles pressurized or dry
— fighters fly low-pressure or dry cockpits with the pilot on suit
pressure, capsules wear suits for launch/entry as backup to sea-level
air. Suit mass rides the anchors like seat mass. `HoseFed` suits depend
on vehicle air (lose the cabin and they lose the loop — future failure
model, not today); `SelfContained` suits are EVA-capable. Implemented as
`Crew.suited/suit_mass_kg_each/suit_type`: suits without mass, and mass
without suits, both refuse. Injury, consciousness, and thermal modeling
of the human are out of scope: the
cabin layer tracks presence, fit, mass, and air — never biology.

## 8. Venting and EVA without an airlock

Cabin pressure is runtime state per pressure volume —
`Pressurized | Vacuum` — owned by the same region that
authors the atmosphere. Venting dumps the tracked air inventory
overboard (mass goes to zero on the gauges); repressurizing consumes
stored air, which makes air a consumable and reserves a future air-tank
part plus vent/repress rate physics (orifice flow, later slice).

Hatch rule (implemented in `sim-core::cabin`): an exterior hatch
opens only into a `Vacuum` region, or into a region whose occupants are
all suited; EVA additionally needs self-contained suits. EVA without an
airlock is exactly Gemini-style whole-cabin venting: suits on, vent,
open, lose the air, repress from reserve on return. An airlock part
(small cycled volume, KSP-style part) avoids dumping the whole cabin
and arrives as its own part slice. Vent/repress rates and the
air-reserve tank part stay future slices.

Connected pressure volumes in an assembled vehicle share gas through open
assembly hatches. The current topology transition solves the ideal-gas
equilibrium immediately, conserving air, oxygen, and sensible thermal
energy under a constant dry-air heat-capacity model. Each cabin reports its
current pressure from its inventory; `pressure_kpa` remains its authored
repressurization target. Finite-rate flow through the hatch opening remains
future work. Vehicle-level `vent_cabin` and `repress_cabin` operations,
along with hatch equalization, update vehicle mass and inertia from each
cabin's air-mass change and recenter all body-frame geometry on the new COM.
Callers should use these vehicle-level operations rather than mutating a
cabin's air inventory directly when the cabin is part of a flown vehicle.

## 9. Control authority (KSP-like, presence-based)

Whether the craft answers the controls is a discrete capability flag,
computed from the vehicle definition plus manifest — a separate graph
from aero, propulsion, pressure, and resources (same separation as the
docking graphs in `01`). Close to KSP: no pilot at a station and no
autopilot core aboard means nobody flies the craft.

- **Sources.** (a) A pilot at a control station: a flight-deck seat or
  a designated pilot couch/cockpit seat, occupied. Presence only for
  now — skill, fatigue, and injury stay future. A crew member on EVA
  does not count (not at a station). (b) An autopilot block: an
  avionics monument with a capability tier — `Hold` (stability
  augmentation only), `Fly` (executes maneuvers), `Full` (runs
  programs). Pilots map to full manual plus augmentation;
  cores map to tiered automation.
- **Dependencies (noted, not implemented).** A core needs electrical
  power (future electrical graph); remotely commanded operation needs a
  comm link (future comm graph). Recorded here so the flag has places
  to plug them in later instead of growing booleans.
- **No source, no control.** FBW emits nothing, manual axes are
  rejected, autopilot graphs cannot arm; the craft continues on last
  trim/ballistic. UI reports the reason (no pilot / no core / no
  power / no comm). Scripts (`docs/18`, §9 there) require authority to
  arm; their schedulers check the flag first.

Implemented presence-based in `sim-core::cabin`/`vehicle`: pilot
stations and core tiers bake from crew regions and avionics cores,
`control_authority()` returns the flag with a reason, and both control
intakes refuse with `NoControlAuthority` unless the asset predates crew
modeling entirely (legacy migration). Power/comm gates arrive with
their graphs.

## 10. Worked examples (illustrative arithmetic)

All numbers below are hand-checkable illustrations, not certification.

**747-class main deck.** Inner width 6.1 m. Block `[3, 4, 3]` economy,
seat 0.44 m, aisles 2 × 0.51 m:
`10 × 0.44 + 2 × 0.51 = 5.42 m`, plus 0.4 m walls/armrests = 5.82 m ≤
6.1 m ✓. 40 rows at 0.81 m pitch = 32.4 m of cabin plus monuments and
5 door pairs. 400 seats need 8 attendant places (400/50) and
exit ratings totalling ≥ 400 (e.g. 5 pairs of the largest type).

**747 upper deck.** Same loft, higher `floor_z_m`, narrower chord:
block `[3, 3]`, `6 × 0.44 + 0.51 = 3.15 m` ✓ with large margin; 16
rows = 96 seats. Headroom is what binds the hump ends, not width.

**Concorde-class.** Outer 2.88 m, inner ~2.5 m. Block `[2, 2]`,
`4 × 0.43 + 0.43 = 2.15 m` + 0.2 m walls = 2.35 m ≤ 2.5 m ✓ — while a
`[3, 3]` block (`6 × 0.43 + 0.43 + 0.2 = 3.21 m`) fails the same check.
25 rows at 0.86 m = 21.5 m for 100 places.

**Fighter tandem.** Two ejection-seat blocks, 1-abreast, ~1.4 m
spacing, seat+rails 110 kg authored each, pilot suited (`HoseFed`,
~20 kg) so the cockpit compiles at low pressure; canopy as a future
cutout (`03`, §9), instruments as avionics manifest. A single-seat
variant with an autopilot core instead of a pilot is controllable
exactly when the core is aboard (§9). Width fit is trivially
satisfied; the binding checks are headroom under the canopy line and
the pressure schedule.

**EVA without airlock (capsule).** Two suited `SelfContained` couches,
cabin vented to `Vacuum`, hatch opens under the §8 rule; the ~2 kg of
cabin air is lost and repress needs reserve. Same parts, no airlock.

## 11. Editor UX (two depths, like fuselages)

- **Simple mode:** passenger count + class mix (+ decks for 747-likes)
  auto-fill the straight section of the loft; fails closed with the
  first violated rule named (width at row x, length, exits, crew).
- **Advanced mode:** decks/blocks/monuments/doors edited directly.
- **Presets** (convenience, same physics): 747-like 3-class double
  deck, Concorde-like 100-seat single class, single/tandem fighter
  cockpit. Capsule presets stay where they are (test/doc parameter
  sets, never library hardcodes).

## 12. Compile targets (decided at implementation)

Direction, not final schema: seat blocks lower into the existing
`Crew` anchor/mass path (one logical row set per block, class tags
added to anchors); monuments lower into manifest mass; doors lower
into door records for exit accounting and future evacuation hooks;
suit flags ride the anchors; the authority flag (§9) lowers into a
`controllable` capability with a reason code. No forked pipeline: the
capsule path must keep compiling unchanged through every slice
(regression-pinned).

## 13. Implementation slices (in order, each with tests)

1. `SeatBlock` with column groups + per-row width fit against the
   loft (hand-width unit tests; `[3,4,3]` passes wide, fails narrow).
2. Classes (preset bundles, authored overrides) + `Monument` mass and
   length packing (overlap/packing refusal tests).
3. `Door` fit + project-owned exit table + exit-limited occupancy +
   pilot/attendant rules (refusal tests).
4. Deck height/headroom + 747-like double-deck golden (main + upper).
5. Suits (implemented): per-block flag/mass/type, pressure exemption,
   unsuited-dry refusal; suited EVA-eligibility tag.
6. Venting state + hatch rule + air-consumable accounting (implemented
   in `sim-core::cabin`); airlock part reserved as its own slice after this.
7. Presence-based `controllable` flag with reason codes (implemented:
   pilot stations, core tiers, intake gate, legacy migration); scripts
   check it before arming.
8. Presets (747/Concorde/fighter) + TOML roundtrip + baker wiring.
9. Later, out of scope here: evacuation hooks, consumables/carts,
   metabolic O2 loop, canopy/window cutouts, vent rates, power/comm
   dependencies of cores.

## 14. Open questions

- Exact exit-type ratings and their regulatory source (locks in
  the doors slice).
- Side-pairing rule for exit capacity (total vs per-side).
- Whether attendant places need jump-seat geometry distinct from
  upright anchors.
- Door cutout interaction with the future subtractive layer.
- Business/first monuments (bars, showers) as mass-only or modelled
  volumes.
- Per-place suit overrides vs per-block uniform suits.
- Vent/repress rate physics and air-reserve tank sizing.
- Core power/comm dependency thresholds and the uncontrollable-UI
  vocabulary.
