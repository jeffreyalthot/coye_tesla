#!/usr/bin/env python3
"""Simulation 3D simplifiée d'un résonateur Tesla sans dépendances externes."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

MU0 = 4 * math.pi * 1e-7
G = 9.81


@dataclass
class Material:
    name: str
    density: float
    conductivity: float
    relative_permeability: float


COPPER = Material("Cuivre", 8960, 5.96e7, 0.999994)
ALUMINUM = Material("Aluminium", 2700, 3.5e7, 1.000022)
STEEL = Material("Acier", 7850, 1.45e6, 100)


@dataclass
class Coil:
    turns: int
    radius: float
    length: float
    wire_diameter: float
    resistance: float


@dataclass
class Plate:
    radius: float
    thickness: float


@dataclass
class Rod:
    length: float
    radius: float
    upward_angle_deg: float


@dataclass
class AluminumLeaf:
    length: float
    width: float
    thickness: float
    fold_stiffness: float
    nozzle_angle_deg: float
    thrust_focus: float


@dataclass
class Assembly:
    plate: Plate
    coil: Coil
    rods: Rod
    leaf: AluminumLeaf
    supply_voltage: float


def solenoid_field_center(coil: Coil, current: float) -> float:
    n = coil.turns / coil.length
    return MU0 * n * current


def field_on_axis_loop(current: float, loop_radius: float, z: float) -> float:
    return (MU0 * current * loop_radius**2) / (2 * (loop_radius**2 + z**2) ** 1.5)


def edge_field_samples(assembly: Assembly, current_rms: float) -> Dict[int, float]:
    b_edge = field_on_axis_loop(current_rms, assembly.coil.radius, assembly.plate.radius)
    return {0: b_edge, 90: b_edge, 180: b_edge, 270: b_edge}


def leaf_force_magnetic(leaf: AluminumLeaf, b_field: float, coupling: float = 0.22) -> float:
    area = leaf.length * leaf.width
    shape_gain = 1.0 + 0.5 * math.sin(math.radians(leaf.nozzle_angle_deg))
    direction_gain = 1.0 + leaf.thrust_focus
    magnetic_pressure = (b_field**2) / (2 * MU0)
    return coupling * shape_gain * direction_gain * magnetic_pressure * area


def compute_masses(assembly: Assembly) -> Dict[str, float]:
    plate_mass = STEEL.density * math.pi * assembly.plate.radius**2 * assembly.plate.thickness

    wire_len = 2 * math.pi * assembly.coil.radius * assembly.coil.turns
    wire_area = math.pi * (assembly.coil.wire_diameter / 2) ** 2
    coil_mass = COPPER.density * wire_len * wire_area

    rod_vol = math.pi * assembly.rods.radius**2 * assembly.rods.length
    rods_mass = 4 * rod_vol * STEEL.density

    square_wire_mass = (8 * assembly.plate.radius) * wire_area * COPPER.density
    leaves_mass = 4 * ALUMINUM.density * assembly.leaf.length * assembly.leaf.width * assembly.leaf.thickness

    total = plate_mass + coil_mass + rods_mass + square_wire_mass + leaves_mass
    return {
        "plaque": plate_mass,
        "bobine": coil_mass,
        "batons": rods_mass,
        "fil_carre": square_wire_mass,
        "feuilles": leaves_mass,
        "total": total,
    }


def integrate_motion(
    total_mass: float,
    force_profile: List[Tuple[float, float, float]],
    dt: float,
    floor_z: float = 0.0,
) -> Tuple[List[Tuple[float, float, float]], Tuple[float, float, float]]:
    vx = vy = vz = 0.0
    x = y = z = 0.0
    positions: List[Tuple[float, float, float]] = []

    for fx, fy, fz in force_profile:
        ax = fx / total_mass
        ay = fy / total_mass
        az = fz / total_mass
        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        x += vx * dt
        y += vy * dt
        z += vz * dt

        if z < floor_z:
            z = floor_z
            if vz < 0:
                vz = 0.0

        positions.append((x, y, z))

    return positions, (vx, vy, vz)


def print_simulation_trace(positions: List[Tuple[float, float, float]], dt: float, samples: int = 12) -> None:
    if not positions:
        return

    stride = max(1, len(positions) // samples)
    print("\n=== Affichage de la simulation (échantillons) ===")
    print(" temps(s) |   X(cm) |   Y(cm) |   Z(cm)")
    print("-----------------------------------------")

    for i in range(0, len(positions), stride):
        x, y, z = positions[i]
        t = (i + 1) * dt
        print(f" {t:7.3f} | {x*100:7.2f} | {y*100:7.2f} | {z*100:7.2f}")

    if (len(positions) - 1) % stride != 0:
        x, y, z = positions[-1]
        t = len(positions) * dt
        print(f" {t:7.3f} | {x*100:7.2f} | {y*100:7.2f} | {z*100:7.2f}")


def build_geometry(assembly: Assembly) -> Tuple[List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    bases = []
    tips = []
    for deg in [0, 90, 180, 270]:
        theta = math.radians(deg)
        bx = assembly.plate.radius * math.cos(theta)
        by = assembly.plate.radius * math.sin(theta)
        bz = 0.0

        tilt = math.radians(assembly.rods.upward_angle_deg)
        horizontal = assembly.rods.length * math.cos(tilt)
        vertical = assembly.rods.length * math.sin(tilt)
        tx = bx + horizontal * math.cos(theta)
        ty = by + horizontal * math.sin(theta)
        tz = vertical

        bases.append((bx, by, bz))
        tips.append((tx, ty, tz))
    return bases, tips


def export_geometry_obj(assembly: Assembly, rods_base: List[Tuple[float, float, float]], rods_tip: List[Tuple[float, float, float]], path: str = "tesla_resonator_geometry.obj") -> None:
    vertices: List[Tuple[float, float, float]] = []
    lines: List[Tuple[int, int]] = []

    circle_pts = []
    for i in range(96):
        phi = 2 * math.pi * i / 95
        circle_pts.append((assembly.plate.radius * math.cos(phi), assembly.plate.radius * math.sin(phi), 0.0))
    base = len(vertices)
    vertices.extend(circle_pts)
    for i in range(len(circle_pts) - 1):
        lines.append((base + i + 1, base + i + 2))

    coil_pts = []
    samples = 400
    for i in range(samples):
        t = (2 * math.pi * assembly.coil.turns) * i / (samples - 1)
        z = -assembly.coil.length / 2 + assembly.coil.length * i / (samples - 1)
        coil_pts.append((assembly.coil.radius * math.cos(t), assembly.coil.radius * math.sin(t), z))
    cbase = len(vertices)
    vertices.extend(coil_pts)
    for i in range(samples - 1):
        lines.append((cbase + i + 1, cbase + i + 2))

    for i in range(4):
        idx = len(vertices)
        vertices.append(rods_base[i])
        vertices.append(rods_tip[i])
        lines.append((idx + 1, idx + 2))

    sq = len(vertices)
    vertices.extend(rods_tip)
    lines.extend([(sq + 1, sq + 2), (sq + 2, sq + 3), (sq + 3, sq + 4), (sq + 4, sq + 1)])

    with open(path, "w", encoding="utf-8") as f:
        f.write("# Tesla resonator geometry\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for l in lines:
            f.write(f"l {l[0]} {l[1]}\n")


def main() -> None:
    assembly = Assembly(
        plate=Plate(0.12, 0.004),
        coil=Coil(220, 0.03, 0.10, 1.0e-3, 3.1),
        rods=Rod(0.08, 0.003, 15),
        leaf=AluminumLeaf(0.24, 0.03, 20e-6, 28, nozzle_angle_deg=40, thrust_focus=0.35),
        supply_voltage=10.0,
    )

    masses = compute_masses(assembly)
    total_mass = masses["total"]
    weight = total_mass * G

    duration = 60.0
    dt = 0.01
    steps = int(duration / dt)
    voltage_step = 0.02
    supply_voltage = assembly.supply_voltage
    force_profile: List[Tuple[float, float, float]] = []
    telemetry: List[Tuple[float, float, float, float]] = []
    lift_started = False

    for _ in range(steps):
        current_rms = supply_voltage / assembly.coil.resistance
        b_center = solenoid_field_center(assembly.coil, current_rms)
        single_leaf_force = leaf_force_magnetic(assembly.leaf, b_center)
        total_lift = 4 * single_leaf_force
        net_fz = total_lift - weight

        # La géométrie est réglée pour orienter la poussée principalement sur l'axe Z.
        net_fx = 0.0
        net_fy = 0.0
        force_profile.append((net_fx, net_fy, net_fz))
        telemetry.append((supply_voltage, current_rms, b_center, net_fz))

        if not lift_started:
            if net_fz > 0:
                lift_started = True
            else:
                supply_voltage += voltage_step

    positions, vel = integrate_motion(total_mass, force_profile, dt, floor_z=0.0)
    final = positions[-1]
    launch_time = 0.0
    for i, (_, _, z) in enumerate(positions):
        if z > 1e-5:
            launch_time = (i + 1) * dt
            break

    start_v, start_i, start_b, _ = telemetry[0]
    end_v, end_i, end_b, end_fz = telemetry[-1]
    b_samples = edge_field_samples(assembly, end_i)

    rods_base, rods_tip = build_geometry(assembly)
    export_geometry_obj(assembly, rods_base, rods_tip)

    print("=== Paramètres électriques/magnétiques ===")
    print(f"Tension départ bobine: {start_v:.2f} V")
    print(f"Tension finale bobine: {end_v:.2f} V")
    print(f"Courant RMS final: {end_i:.3f} A")
    print(f"Champ départ centre bobine: {start_b*1e3:.2f} mT")
    print(f"Champ final centre bobine: {end_b*1e3:.2f} mT")
    for d in [0, 90, 180, 270]:
        print(f"Champ sur bord plaque à {d:>3}°: {b_samples[d]*1e3:.3f} mT")

    print("\n=== Masses des composants ===")
    for k, v in masses.items():
        print(f"{k:>10}: {v:.4f} kg")

    print("\n=== Forces et déplacement ===")
    print(f"Force magnétique finale (1 feuille): {leaf_force_magnetic(assembly.leaf, end_b):.3f} N")
    print(f"Force magnétique finale totale: {weight + end_fz:.3f} N")
    print(f"Poids total: {weight:.3f} N")
    print(f"Force nette finale (x,y,z): (0.000, 0.000, {end_fz:.3f}) N")
    print(f"Vitesse finale (x,y,z): ({vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f}) m/s")
    print(f"Distance parcourue: X={final[0]*100:.2f} cm, Y={final[1]*100:.2f} cm, Z={final[2]*100:.2f} cm")
    print(f"Décollage observé à t={launch_time:.2f} s")
    print(f"Durée de simulation: {duration:.1f} s")
    print_simulation_trace(positions, dt=dt)
    print("Géométrie 3D exportée dans tesla_resonator_geometry.obj")


if __name__ == "__main__":
    main()
