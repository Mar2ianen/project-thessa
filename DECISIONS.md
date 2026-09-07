# Design decisions — current baseline

Краткая индексная версия ADR. Если решение меняется, старое не переписывается молча: добавляется новый ADR и пометка `superseded`.

| ID | Решение | Статус |
|---|---|---|
| ADR-0001 | Небесные тела используют baked deterministic ephemerides; корабли — full multi-body test particles | accepted |
| ADR-0002 | Bevy — client/app shell; `sim-core` не зависит от Bevy | accepted |
| ADR-0003 | Server-authoritative multiplayer + shared warp | accepted |
| ADR-0004 | Параметрическая конструкция компилируется в несколько физических представлений | proposed |
| ADR-0005 | x86_64 release: AVX2 baseline + отдельный AVX-512 target | proposed |
| ADR-0006 | Physics ray queries имеют CPU/BVH canonical path; hardware RT не является обязательным источником истины | proposed |
| ADR-0007 | Reusable engine crates — MIT; game code/apps — GPL-3.0-or-later | accepted |
| ADR-0008 | Cross-platform from day one; no DirectX-facing domain/gameplay API; rendering through Bevy/wgpu | accepted |
| ADR-0009 | Autopilot UX uses MechJeb-like high-level actions, composed as typed event-driven graphs with fork/join | accepted |

См. `docs/adr/`.
