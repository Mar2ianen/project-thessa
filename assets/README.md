# Prototype visual assets

В этой папке лежат временные визуальные ассеты для Bevy-клиента:

- `textures/khepri-surface-v1.png` — горячая вулканическая поверхность Khepri;
- `textures/janus-surface-v1.png` — холодная rocky surface Janus;
- `textures/orthea-surface-v1.png` — холодная тяжёлая поверхность Orthea;
- `textures/pelagos-surface-v1.png` — океаническая поверхность Pelagos;
- `textures/thessa-surface-v1.png` — поверхность луны Thessa;
- `textures/thessa-surface-v2.png` — Thessa v0.2: процедурная текстура 16K
  (16384x8192) из `thessa-worldgen-rocky` (`export-client-texture` из
  spec-рецепта, сид 7, стриминговый собственный PNG-кодер).
  Перегенерируется детерминированно, v1 оставлен для отката;
- `textures/borea-surface-v1.png` — ледяная/базальтовая поверхность Borea;
- `textures/nereid-atmosphere-v1.png` — тёплые полосы атмосферы газового гиганта Nereid.
- `textures/vesper-atmosphere-v1.png` — холодные полосы атмосферы Vesper.

Все texture maps сгенерированы встроенным ImageGen 2026-09-07 по отдельным
описаниям для прототипа. Это не физически точные карты поверхности и не часть
MIT/GPL лицензионной границы исходного кода. Перед публичным релизом ассетов
нужно зафиксировать отдельную лицензию и provenance каждого файла.
