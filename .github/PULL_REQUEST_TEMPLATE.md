## Что меняется

<!-- Коротко опишите изменение и зачем оно нужно. -->

## Проверки

- [ ] `cargo fmt --all -- --check`
- [ ] `cargo clippy --workspace --all-targets --all-features -- -D warnings`
- [ ] `cargo test --workspace`
- [ ] Проверены численные tolerances / conserved quantities, если менялась физика
- [ ] Обновлены docs/ADR, если изменился контракт или лицензирование

## Лицензия и границы

- [ ] Новые зависимости совместимы с лицензией целевого crate
- [ ] Код не протекает из GPL game crates в MIT engine crates
- [ ] Platform-specific API не попал в domain/simulation code
