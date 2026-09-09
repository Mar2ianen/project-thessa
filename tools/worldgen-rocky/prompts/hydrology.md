# Hydrology (water + ice mask, channel-packed)

- R channel: ocean (deep + shelf), 0 = dry land, 255 = open water
- G channel: lakes / rivers / wet sediment, 0 = dry, 255 = water
- B channel: ice / snow cover (permanent + seasonal), 0 = none, 255 = iced

Land stays `#000000` except ice (blue tint). Ocean mask must match biome
ocean colors and height values below datum. No clouds, no wave whitecaps.
