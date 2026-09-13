"""Small adapters for installed reference solvers.

The file is executed by the Rust validation binary with ``python -c``.  It is
kept outside the runtime graph: no reference package is imported by
``thessa-sim-core`` or the game crates.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import math


LBF_TO_N = 4.4482216152605
PSF_TO_PA = 47.88025898033584
FT2_TO_M2 = 0.09290304
FT_TO_M = 0.3048


def jsbsim_result(model_name: str, mach: float, alpha_deg: float) -> tuple[float, ...]:
    import jsbsim

    # The bundled models are educational reference aircraft, not claims of
    # flight-test accuracy.  Keep the point static and use a non-grounded
    # altitude so only the configured aero model contributes to the result.
    fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
    fdm.set_debug_level(0)
    fdm.set_dt(0.01)
    capture = io.StringIO()
    with contextlib.redirect_stdout(capture), contextlib.redirect_stderr(capture):
        if not fdm.load_model(model_name):
            raise RuntimeError(f"JSBSim could not load model {model_name}")
        altitude_ft = 80_000.0 if mach > 1.2 else 10_000.0
        initial_conditions = {
            "ic/h-sl-ft": altitude_ft,
            "ic/mach": mach,
            "ic/alpha-deg": alpha_deg,
            "ic/beta-deg": 0.0,
            "ic/gamma-deg": 0.0,
            "ic/theta-deg": alpha_deg,
            "ic/phi-deg": 0.0,
            "ic/psi-true-deg": 0.0,
        }
        for property_name, value in initial_conditions.items():
            fdm.set_property_value(property_name, value)
        if not fdm.run_ic():
            raise RuntimeError(f"JSBSim could not initialize model {model_name}")

    q_pa = fdm.get_property_value("aero/qbar-psf") * PSF_TO_PA
    reference_area_m2 = fdm.get_property_value("metrics/Sw-sqft") * FT2_TO_M2
    reference_length_m = fdm.get_property_value("metrics/cbarw-ft") * FT_TO_M
    drag_n = fdm.get_property_value("forces/fwx-aero-lbs") * LBF_TO_N
    lift_n = fdm.get_property_value("forces/fwz-aero-lbs") * LBF_TO_N
    pitch_moment_nm = fdm.get_property_value("moments/m-aero-lbsft") * LBF_TO_N * FT_TO_M
    denominator = q_pa * reference_area_m2
    if denominator <= 0.0 or reference_length_m <= 0.0:
        raise RuntimeError(f"JSBSim returned an invalid reference scale for {model_name}")
    return (
        fdm.get_property_value("velocities/mach"),
        fdm.get_property_value("aero/alpha-deg"),
        lift_n / denominator,
        drag_n / denominator,
        pitch_moment_nm / (denominator * reference_length_m),
        reference_area_m2,
        reference_length_m,
    )


def jsbsim_trajectory_result(
    model_name: str, mach: float, alpha_deg: float, duration_s: float
) -> tuple[float, ...]:
    import jsbsim

    fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
    fdm.set_debug_level(0)
    dt_s = 0.01
    fdm.set_dt(dt_s)
    capture = io.StringIO()
    with contextlib.redirect_stdout(capture), contextlib.redirect_stderr(capture):
        if not fdm.load_model(model_name):
            raise RuntimeError(f"JSBSim could not load model {model_name}")
        for property_name, value in {
            "ic/h-sl-ft": 80_000.0,
            "ic/mach": mach,
            "ic/alpha-deg": alpha_deg,
            "ic/beta-deg": 0.0,
            "ic/gamma-deg": 0.0,
            "ic/theta-deg": alpha_deg,
            "ic/phi-deg": 0.0,
            "ic/psi-true-deg": 0.0,
        }.items():
            fdm.set_property_value(property_name, value)
        if not fdm.run_ic():
            raise RuntimeError(f"JSBSim could not initialize model {model_name}")
        initial_speed_mps = fdm.get_property_value("velocities/vt-fps") * FT_TO_M
        for _ in range(round(duration_s / dt_s)):
            if not fdm.run():
                raise RuntimeError(f"JSBSim stopped during {model_name} trajectory")
    return (
        fdm.get_property_value("velocities/mach"),
        fdm.get_property_value("velocities/vt-fps") * FT_TO_M,
        fdm.get_property_value("position/h-sl-ft") * FT_TO_M,
        fdm.get_property_value("aero/alpha-deg"),
        fdm.get_property_value("aero/qbar-psf") * PSF_TO_PA,
        initial_speed_mps,
    )


def jsbsim_polar(model_name: str) -> list[tuple[float, ...]]:
    rows = []
    for mach in (0.95, 2.0):
        for alpha_deg in (-5.0, 5.0):
            values = jsbsim_result(model_name, mach, alpha_deg)
            rows.append((mach, alpha_deg, values[2], values[3], values[4]))
    return rows


def rocketpy_fin_result(mach: float) -> tuple[float, ...]:
    from rocketpy import Rocket

    rocket = Rocket(
        radius=0.25,
        mass=10.0,
        inertia=(1.0, 1.0, 1.0),
        power_off_drag=0.0,
        power_on_drag=0.0,
        center_of_mass_without_motor=2.0,
    )
    rocket.add_nose(length=0.75, kind="von karman", position=4.0)
    fins = rocket.add_trapezoidal_fins(
        n=4,
        root_chord=0.5,
        tip_chord=0.2,
        span=0.25,
        position=1.5,
    )
    # RocketPy stores cpz in the fin-set local frame. Convert it to the
    # rocket's global tail-to-nose coordinate before comparing with Thessa's
    # body-frame panel moment arm.
    global_cp_m = 1.5 - float(fins.cpz)
    return float(fins.clalpha(mach)), global_cp_m


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, choices=("jsbsim", "rocketpy"))
    parser.add_argument("--model", default="")
    parser.add_argument("--mach", type=float, required=True)
    parser.add_argument("--alpha-deg", type=float, default=0.0)
    parser.add_argument("--trajectory", action="store_true")
    parser.add_argument("--polar", action="store_true")
    parser.add_argument("--duration-s", type=float, default=5.0)
    args = parser.parse_args()

    if args.reference == "jsbsim":
        if args.trajectory:
            values = jsbsim_trajectory_result(
                args.model, args.mach, args.alpha_deg, args.duration_s
            )
            print(
                "trajectory_result,JSBSim,{}, {:.9g},{:.12g},{:.12g},{:.12g},{:.12g}".format(
                    args.model,
                    values[0],
                    values[1],
                    values[2],
                    values[3],
                    values[4],
                ).replace(" ", "")
            )
            return
        if args.polar:
            for mach, alpha_deg, lift, drag, pitch_moment in jsbsim_polar(args.model):
                print(
                    "polar,JSBSim,{}, {:.9g},{:.9g},{:.12g},{:.12g},{:.12g}".format(
                        args.model,
                        mach,
                        alpha_deg,
                        lift,
                        drag,
                        pitch_moment,
                    ).replace(" ", "")
                )
            return
        values = jsbsim_result(args.model, args.mach, args.alpha_deg)
        print(
            "result,JSBSim,{}, {:.9g},{:.9g},{:.12g},{:.12g},{:.12g},nan,nan".format(
                args.model,
                values[0],
                values[1],
                values[2],
                values[3],
                values[4],
            ).replace(" ", "")
        )
    else:
        cl_alpha, global_cp_m = rocketpy_fin_result(args.mach)
        print(
            "result,RocketPy,barrowman_fin_set,{:.9g},{:.9g},nan,nan,nan,{:.12g},{:.12g}".format(
                args.mach,
                args.alpha_deg,
                cl_alpha,
                global_cp_m,
            )
        )


if __name__ == "__main__":
    main()
