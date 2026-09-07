# 01 — Небесная система

Статус: **design baseline v0.1**. Названия рабочие. Числа предназначены для прототипа и последующего numerical validation, а не как финальный lore canon.

## 1.1. Главная идея

Система — иерархическая тройная:

```text
                 B ───── C
              тесная двойная
                    ●  BC barycenter
                    │
                    │  outer relative orbit ~45 AU
                    │
                    A
               наша K-звезда
              /    |      \
        Khepri   Nereid   Orthea — Vesper
                   │
              resonant moons
```

Строго все три компонента обращаются вокруг общего барицентра, но `M_B + M_C > M_A`, поэтому визуально и динамически система A с её планетами — внешний компонент вокруг массивной тесной двойной.

### Runtime policy

- canonical celestial bodies **не интегрируются** в runtime;
- их state задаётся функцией `body_state(body_id, sim_time)` из baked ephemerides;
- offline `system-baker` обязан строить/валидировать устойчивую конфигурацию;
- корабли используют фактические положения всех релевантных тел и получают multi-body gravity;
- baked body motion не означает patched conics для кораблей.

---

## 1.2. Звёзды

| Объект | Класс/роль | Масса | Радиус | `T_eff` | Светимость | Цветовой характер |
|---|---|---:|---:|---:|---:|---|
| **Asterion A** | K-type, наша звезда | 0.82 M☉ | 0.83 R☉ | 5100 K | 0.42 L☉ | жёлто-оранжевый |
| **Asterion B** | горячая компонента BC | 2.00 M☉ | 1.75 R☉ | 8700 K | 15.8 L☉ | бело-голубой |
| **Asterion C** | холодная компонента BC | 0.38 M☉ | 0.39 R☉ | 3500 K | 0.020 L☉ | красно-оранжевый |

Возраст системы **не является ограничением дизайна**. Требование — правдоподобное текущее состояние и динамическая устойчивость на выбранном horizon, а не reconstruction formation history.

### Внутренняя орбита B–C

| Параметр | Значение |
|---|---:|
| relative semi-major axis | 0.220 AU |
| eccentricity | 0.040 |
| period | 24.431 d |
| mutual inclination к outer plane | ~0° (почти coplanar) |

Плоскость внутренней пары намеренно почти совпадает с плоскостью внешней орбиты. Поэтому с объектов системы A пара может быть **eclipsing binary**: C периодически проходит по диску B, B периодически закрывает C. Это настоящий геометрический эффект baked ephemerides, не skybox animation.

Максимальное видимое разделение при ~45 AU порядка `0.28°`, то есть две компоненты могут читаться как две отдельные яркие точки с разным цветом.

### Внешняя орбита A ↔ BC

| Параметр | Значение |
|---|---:|
| relative semi-major axis | 45.0 AU |
| eccentricity | 0.060 |
| relative period | 168.75 y |
| periapsis distance | 42.3 AU |
| apoapsis distance | 47.7 AU |

При такой конфигурации грубая Holman–Wiegert S-type оценка устойчивой circumstellar зоны вокруг A даёт `a_crit ≈ 7.64 AU`. Самая внешняя крупная планета A ниже находится на 3.8 AU — с большим игровым запасом.

### Свет B+C в системе A

У Nereid (`a=0.78 AU` от A):

- поток от A: `0.42 / 0.78² ≈ 0.690` земного солнечного потока;
- поток от B+C на средней внешней дистанции: `(15.8 + 0.020) / 45² ≈ 0.00781`;
- отношение: **~1.13%** от прямого света A;
- из-за outer eccentricity диапазон около **1.0–1.28%**.

Это не основной industrial power source, но уже реальный резерв для avionics, pumps, cryogenics, communications и low-power survival режимов. C энергетически почти не важна, но важна визуально и для спектрального состава.

---

## 1.3. Планеты вокруг Asterion A

| Объект | Тип | `a` | `e` | `i` | Период | Масса | Радиус | Роль |
|---|---|---:|---:|---:|---:|---:|---:|---|
| **Khepri** | горячая каменная | 0.220 AU | 0.030 | 1.2° | 41.62 d | 0.35 M⊕ | 4400 km | high-T / sulfur / refractory mining |
| **Nereid** | тёплый gas giant | 0.780 AU | 0.015 | 0.4° | 277.86 d | 0.95 MJ | 68 000 km | главная moon system, H/He/He-3 late game |
| **Orthea** | холодная super-Earth | 2.150 AU | 0.025 | 1.1° | 1271.57 d | 2.40 M⊕ | 9000 km | тяжёлый surface world + своя moon logistics |
| **Vesper** | внешний ice giant | 3.800 AU | 0.030 | 2.0° | 2987.85 d | 0.12 MJ | 36 000 km | дальняя volatile/cryogenic logistics |

### Khepri

**Поверхность:** `~0.73 g`, escape `~7.96 km/s`.

**Атмосфера (design target):**

- `0.04 bar`;
- CO₂-dominant, SO₂/Ar/N₂ traces;
- горячая, пыльная, химически неприятная;
- средняя surface temperature ориентировочно 520–650 K в зависимости от региона.

**Основные ресурсы:** sulfur, refractory metals, nickel-group ores, iron, silicates, trace fissile minerals.

**Gameplay:** поздний heat-material world; очень сильная solar generation от A, но тяжёлая surface thermal engineering.

### Nereid

**Cloud-top gravity:** порядка `2.65 g`; escape velocity у reference radius `~59.5 km/s`.

**Bulk atmosphere:** H₂/He с methane/ammonia/water traces. Полноценной «поверхности» нет.

**Основные ресурсы:**

- hydrogen;
- helium-4;
- deuterium-bearing hydrogen;
- trace helium-3 — поздний стратегический ресурс;
- atmospheric carbon/nitrogen chemistry.

**Кольца (working):** `85 000–140 000 km` от центра, тонкие и светлые; лёд + пыль + silicates. Кольца — и ресурс, и visual/lighting system.

### Orthea

**Поверхность:** `~1.20 g`, escape `~14.58 km/s`.

**Атмосфера:** design target `~2.8 bar`, N₂/CO₂/Ar; холодная, сильный greenhouse, но средняя температура всё равно примерно 200–230 K. Не Earth-clone.

**Основные ресурсы:** iron, nickel/cobalt, fissile ores, silicates, buried water, carbonates; локальные aluminium ores.

**Gameplay:** выход с поверхности дорогой, поэтому её маленькие ледяные луны становятся естественными fuel depots.

### Vesper

Ice giant без твёрдой доступной поверхности. H₂/He/CH₄ atmosphere; outer-system volatile source. Основной gameplay идёт через moons + upper-atmosphere operations в late game.

---

## 1.4. Nereid — главная система спутников

Пять больших регулярных спутников образуют **номинальную period chain `1:2:4:8:16`**. Табличные периоды — design targets. Canonical ephemeris должна быть получена offline capture/relaxation run и может отличаться на доли процента, сохраняя librating resonant angles.

| Луна | `a` от Nereid | `P` | `e` | Масса | Радиус | Surface g | Escape | Состояние |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| **Pyra** | 398 358 km | 40 h | 0.006 | 0.006 M⊕ | 1050 km | 0.22 g | 2.13 km/s | tidal lock |
| **Thessa** | 632 354 km | 80 h | 0.003 | 0.130 M⊕ | 3200 km | 0.52 g | 5.69 km/s | tidal lock |
| **Pelagos** | 1 003 799 km | 160 h | 0.004 | 0.055 M⊕ | 2500 km | 0.36 g | 4.19 km/s | tidal lock |
| **Auron** | 1 593 432 km | 320 h | 0.002 | 0.020 M⊕ | 1700 km | 0.28 g | 3.06 km/s | tidal lock |
| **Borea** | 2 529 415 km | 640 h | 0.005 | 0.022 M⊕ | 2100 km | 0.20 g | 2.89 km/s | tidal lock |

Nereid Hill radius на его орбите вокруг A: примерно **8.24 million km**. Все пять regular moons находятся глубоко внутри консервативной prograde-зоны.

### Pyra

Вулканический/приливно нагретый inner moon.

- atmosphere: почти отсутствует, локальные SO₂ exospheres;
- mean surface: экстремально неоднородная;
- resources: sulfur, iron, nickel-group, refractory minerals, geothermal heat;
- gameplay: high-temperature materials, dangerous automated mining, early source sulfur outside Thessa.

### Thessa — стартовый мир

**Это главный стартовый biome/world, но не “Kerbin clone”.**

- radius: 3200 km;
- mass: 0.13 M⊕;
- gravity: ~0.52 g;
- surface circular velocity: ~4.02 km/s;
- escape: ~5.69 km/s;
- atmosphere: target `1.08 bar`, примерно N₂ 76%, O₂ 21%, Ar/CO₂/прочее 3%; composition provisional;
- mean climate: ~280–292 K с региональной влажностью/океанами;
- breathable status/life: **open question**, не фиксируется v0.1.

Design target для ascent: типичный нормальный химический аппарат должен требовать примерно **4.6–5.1 km/s effective Δv** до low orbit в зависимости от аэродинамики/профиля. Это намеренно сложнее stock-KSP-подобного мира, но далеко от Earth/RSS.

**Стартовые ресурсы:** iron, copper, silicates/limestone, quartz, water, carbon feedstock. Sulfur ограничен. High-grade aluminium/nickel/fissile resources намеренно скудны, чтобы космос имел экономическую причину.

### Pelagos

Ocean moon.

- atmosphere: ~1.7 bar, N₂-rich, CO₂/H₂O/Ar;
- surface: океаны + архипелаги;
- climate: тёплый/влажный;
- resources: water at industrial scale, dissolved salts, lithium-bearing brines, deuterium feedstock, carbon/nitrogen chemistry;
- gameplay: floating industry, sea transport, fuel/chemical hub.

### Auron

Сухой metal-rich moon.

- atmosphere: trace `~0.006 bar` CO₂/Ar;
- resources: bauxite/aluminium ore, titanium, nickel/cobalt, uranium/thorium, quartz;
- gameplay: главный early/mid-game reason построить регулярную межлунную heavy logistics.

### Borea

Cold volatile moon.

- atmosphere: target `~0.55 bar`, N₂/CH₄/Ar;
- high-albedo ice regions, methane/nitrogen cryochemistry;
- resources: nitrogen volatiles, methane/carbon feedstock, water ice, ammonia compounds;
- gameplay: cryogenic industry, distant logistics, host of stable-ish submoon.

### Nix — сублуна Borea

Рабочий диаметр **25 km** (`R = 12.5 km`).

| Параметр | Значение |
|---|---:|
| orbit around Borea | 14 000 km from center |
| eccentricity | 0.003 |
| inclination | 2° |
| period | 30.87 h |
| density target | 2.2 g/cm³ |
| atmosphere | none |

Borea Hill radius вокруг Nereid: ~73 250 km. Low-e prograde test-particle stability limit по `~0.4895 R_H` — ~35 900 km; Nix на 14 000 km имеет большой instantaneous dynamical margin. Long-term tidal survival всё равно надо отдельно валидировать через `Q/k2` model.

**Resources:** water ice + dirty rock. Gameplay: дешёвый orbital propellant source / depot и красивый вложенный уровень логистики.

### Halo — co-orbital minor body

Рабочий `R ~30 km`, ледяно-пылевой объект около `L4` Borea на той же большой орбите. Не progression-critical. Нужен как тест/контент для co-orbital dynamics и low-gravity mining.

### Cinder — irregular retrograde moon

- `R ~65 km`;
- `a ~5.8 million km`;
- `e ~0.08`;
- inclination ~162°;
- period ~92.6 d;
- atmosphere: none;
- resources: platinum-group / dense metal-rich material.

Он намеренно outer/retrograde. Это **validation-sensitive** объект: final ephemeris принимается только после long-horizon integration.

---

## 1.5. Hohmann-like окна внутри Nereid system

Ни одно время рейса не является «таймером маршрута». Ниже — reference для двухимпульсного Hohmann transfer из номинальной круговой орбиты Thessa вокруг Nereid, без учёта moon capture burn и multi-body perturbations.

| Thessa → | Transfer time | Synodic repeat / грубое окно | Target phase at departure |
|---|---:|---:|---:|
| **Pyra** | 29.43 h | 80.00 h | −84.9° |
| **Pelagos** | 58.86 h | 160.00 h | +47.6° |
| **Auron** | 93.39 h | 106.67 h | +74.9° |
| **Borea** | 158.11 h | 91.43 h | +91.1° |

Из-за резонансной архитектуры окна регулярны и хорошо подходят для industrial timetable. При этом gravity-assist chains через внешние/внутренние moons могут давать более дешёвые, но более длинные маршруты. Planner должен показывать **несколько Pareto-вариантов**: time / propellant / risk / window frequency.

---

## 1.6. Planetshine и multi-source light на Thessa

Угловой диаметр Nereid с Thessa:

`2 asin(68 000 / 632 354) ≈ 12.3°`.

При geometric albedo `~0.55` full-phase planetshine design estimate порядка:

`0.55 * (68 000 / 632 354)^2 ≈ 0.64%` прямого света A.

Внешняя BC-пара даёт ещё ~1.0–1.28% A в зависимости от outer orbital phase.

Следствие: на night side Thessa при удачной геометрии панели могут получать **порядка процентов дневной мощности**, а не ноль. Точная мощность должна учитывать spectrum response панели и actual phase/occlusion.

При tidal lock Nereid почти фиксирован в небе Thessa:

- sub-Nereid hemisphere: гигант всегда виден;
- anti-Nereid hemisphere: гигант никогда не виден;
- фаза Nereid меняется с 80 h orbital period Thessa;
- eclipses A by Nereid — реальное periodic event;
- B и C остаются независимыми источниками света.

---

## 1.7. Спутники Orthea

Orthea имеет не декоративные луны, а локальную logistics system.

| Объект | `a` | `P` | Размер | Атмосфера | Основная роль |
|---|---:|---:|---:|---|---|
| **Koro** | 55 000 km | 23.02 h | R 18 km | none | inner ice/propellant rock |
| **Mira** | 190 000 km | 147.79 h | R 1450 km, 0.012 M⊕ | ~0.08 bar N₂/CO₂ | полноценная moon base, metals/water |
| **Dey** | 420 000 km | 485.71 h | R 10 km | none | outer dirty-ice depot |

Orthea Hill radius порядка **4.49 million km**, поэтому эта система очень глубоко внутри допустимой области.

Koro/Dey — сознательный Phobos/Deimos-like gameplay: на тяжёлой super-Earth вода дорогая в подъёме, а рядом есть маленькие low-escape ледяные склады.

---

## 1.8. Спутники Vesper

| Объект | `a` | `P` | Размер | Атмосфера | Ресурсы/роль |
|---|---:|---:|---:|---|---|
| **Skadi** | 210 000 km | 43.08 h | R 720 km, ~0.002 M⊕ | trace N₂/CH₄ | volatiles, ice, remote mine |
| **Mote** | 520 000 km | 167.85 h | R 25 km | none | tiny fuel/ice depot |

Vesper Hill radius порядка **19.8 million km** — большой запас.

---

## 1.9. Мир вокруг центральной двойной: Janus

BC не пустая декорация late game.

### Janus

| Параметр | Значение |
|---|---:|
| type | circumbinary super-Earth |
| host | BC barycenter |
| semi-major axis | 4.40 AU |
| eccentricity | 0.020 |
| inclination | 2° |
| period | 2185.15 d (~5.98 y) |
| mass | 3.20 M⊕ |
| radius | 9800 km |
| gravity | ~1.35 g |
| escape | ~16.13 km/s |

Суммарный stellar flux B+C на 4.4 AU: `(15.82 / 4.4²) ≈ 0.817` Earth solar constant — удобно для заметно иного, но не обязательно мёртвого мира.

**Atmosphere target:** ~1.4 bar N₂/CO₂/Ar, composition intentionally not Earth-like; climate ~260–290 K depending greenhouse/latitude. Native life — open question.

**Resources:** полный базовый mineral set, редкие высокотемпературные/магнитные материалы, water; не должен быть просто «богаче всего», его ценность — новая звёздная среда и endgame logistics.

### Stability envelope

Для B–C (`a_bin=0.22 AU`, `e=0.04`) грубая Holman–Wiegert P-type critical inner boundary: **~0.51 AU**. Janus на 4.4 AU далеко снаружи.

Если A рассматривать как внешний companion на 45 AU, грубая S-type outer safe scale вокруг BC порядка **15.2 AU**. Janus далеко внутри. То есть 4.4 AU — сознательно скучный по стабильности выбор.

### Mora — луна Janus

| Параметр | Значение |
|---|---:|
| radius | 1700 km |
| mass | 0.025 M⊕ |
| orbit | 210 000 km |
| period | 148.72 h |
| eccentricity | 0.006 |
| atmosphere | trace ~0.02 bar CO₂/N₂ |

Resources: silicates, water ice, metals. Gameplay: local off-world industry вокруг тяжёлой Janus.

---

## 1.10. Primary raw resource families

Не пытаться сделать periodic table simulator. Design target — **около десятка сырьевых семейств**, сложность идёт из processing/geography/logistics.

1. iron ore;
2. copper ore;
3. bulk silicates/carbonates;
4. quartz/silica;
5. carbon feedstock / hydrocarbons;
6. sulfur;
7. aluminium ore;
8. nickel/cobalt/refractory ore;
9. fissile ore (U/Th family);
10. water/ice;
11. nitrogen-rich volatiles;
12. atmospheric H/He stream (late-game special source, а не обычный ground node).

Deuterium — separation product water/hydrogen. Tritium — bred from lithium-bearing feedstock. He-3 — trace separation product Nereid atmosphere.

Полная матрица — `data/resources.toml`.

---

## 1.11. Что надо провалидировать до canon lock

1. Сгенерировать не идеальные Kepler ratios, а устойчивый near-resonant state для пяти moons Nereid.
2. Прогнать offline long-horizon integration минимум по нескольким наборам tolerances.
3. Проверить Cinder и Halo отдельно.
4. Проверить Nix с tidal migration model (`Q`, `k2`, spin evolution Borea).
5. Зафиксировать `epoch 0`, orbital planes, arguments, mean longitudes и eclipse geometry B/C.
6. После этого экспортировать canonical ephemeris в компактное deterministic representation (analytic elements + periodic terms либо Chebyshev segments).
7. Runtime таблицы не должны тихо меняться при изменении integrator версии: ephemeris — versioned game content.
