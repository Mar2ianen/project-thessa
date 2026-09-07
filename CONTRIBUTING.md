# Contributing — draft

Проект пока на стадии design/prototype. До появления первого вертикального среза изменения оцениваются по трём вопросам:

1. усиливает ли это основной цикл `factory -> logistics -> aerospace -> factory`;
2. сохраняет ли это физическую причинность вместо скрытых игровых бонусов;
3. не связывает ли это simulation core с конкретным renderer/network runtime.

## Минимальные требования к изменениям симуляции

- единицы — SI внутри authoritative state;
- каждый новый solver имеет тесты на известные частные случаи;
- никакой физический hot path не должен зависеть от wall-clock time;
- результат не должен зависеть от порядка итерации hash-map;
- новые зависимости проходят проверку лицензии и причины добавления;
- performance claims подтверждаются benchmark/trace, а не ощущением.

## CI baseline (план)

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p sim-core --release
```

Отдельные numerical regression tests должны сравнивать не `==` для float, а физически осмысленные tolerances / conserved quantities.


## Cross-platform requirements

- новые platform APIs сначала проходят через adapter crate/module;
- domain/sim code не принимает `windows`/DirectX types;
- shader feature не принимается, если нет понятного Vulkan/Metal/WebGPU пути или graceful fallback;
- после появления targets CI должен иметь native Linux build и compile/smoke checks для Windows/macOS/WASM.

## Licensing requirements

- MIT engine crate не может зависеть от GPL game crate;
- каждый package имеет явный SPDX `license`;
- новый copyleft dependency требует ADR;
- копирование reference implementation кода в MIT engine запрещено без совместимой лицензии; идеи/алгоритмы переписываются независимо с тестами против публичных результатов.
