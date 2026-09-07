# ADR-0008 — Cross-platform renderer boundary

Status: accepted.

## Decision
Cross-platform support exists from the first prototype. Domain/simulation/gameplay code contains no DirectX-specific APIs. Bevy/wgpu is the rendering boundary. Linux/Vulkan, macOS/Metal, Windows via wgpu backend, and Web/WASM WebGPU are architecture targets.

Direct D3D12/DXR integration is disallowed by default. Internal backend choice made by wgpu is not exposed to simulation/game code.
