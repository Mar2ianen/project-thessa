# Vehicle part commands

`VehiclePartCommand` is the shared discrete-control API for pilot inputs and
future staging/action-group dispatch. Producers send commands; the authoritative
flight runtime applies them to installed subsystem state. Command ordering is
preserved on the flight input channel.
Pilot keyboard and HUD controls apply commands immediately to local prediction
and queue the same typed events for the embedded authoritative server. Ordinary
full-state packets do not reapply unchanged legacy echoes, so a stale echo cannot
undo a discrete part command. `Stage` and `Engine` edge commands retain their
explicit takeover and throttle semantics.

Current commands cover:

- RCS enable/disable;
- reaction-wheel enable/disable;
- per-bank reaction-wheel enable/disable by authored name;
- landing-gear-group deploy/retract, plus per-leg and retractable wheel-chassis
  commands by authored name;
- parachute-group arm/disarm;
- one named parachute's arm, disarm, or cut transition;
- one named engine or chamber's throttle;
- exact propellant transfer between compatible, connected tanks.

Engine activation continues to use the existing typed `Command::Engine` wire
command. `Command::Stage` retains its current slice behavior and does not yet
resolve a stage definition into part commands. Future stage and action-group
bindings should emit these same typed commands rather than implementing their
own subsystem state changes.

Subsystem state machines remain authoritative: gear actuators advance toward
their deployment command, parachutes still wait for their authored opening
conditions after arming, and all loads are evaluated by the physics runtime.
Commands are explicit state-setting operations so duplicated delivery does not
invert a toggle.
