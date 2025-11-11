
import math
from typing import Dict, Tuple
import localization as lx

global P
global t
P=lx.Project(mode='3D',solver='LSE')
t, label = P.add_target()

def solve_position(d1: float, d2: float, d3: float, z_sign: int = +1, eps: float = 1e-9, L2: float = 0, L3: float = 0) -> Tuple[float, float, float]:
    """
    Трилатерация по 3 якорям:
      A1=(0,0,0), A2=(L2,0,0), A3=(0,L3,0).
    Вход: d1,d2,d3 — расстояния до A1,A2,A3 (в метрах).
    Параметр z_sign: +1 — выбрать верхнее решение (z>=0), -1 — нижнее (z<=0).
    Возвращает (x,y,z).
    """
    if L2 <= 0 or L3 <= 0:
        raise ValueError("L2 и L3 должны быть положительными")

    x = (L2*L2 + d1*d1 - d2*d2) / (2.0 * L2)
    y = (L3*L3 + d1*d1 - d3*d3) / (2.0 * L3)

    z2 = d1*d1 - x*x - y*y
    # if z2 < -eps:
    #     raise ValueError(
    #         f"Inconsistent distances: negative z^2={z2:.6g}. "
    #         "Проверьте измерения/калибровку/шум."
    #     )
    if z2 < 0:
        z2 = 0.0

    z = math.sqrt(z2) * (1 if z_sign >= 0 else -1)
    return x, y, z

def calibrated_linear(k, b, x):
    return k * x + b

def calibrated_quad(a, b, c, x):
    return a * x**2 + b * x + c 

def calibrated_qubic(a, b, c, d, x):
    return a * x**3 + b * x**2 + c * x + d


def multilateration(
    raw_data: Dict[int, float], anchor_positions: Dict[int, Tuple[float, float, float]], z_sign: int = +1) -> Tuple[float, float, float]:
    """
    Trilateration algorithm
    :param raw_data: raw data from the sensor
    :param anchor_positions: anchor positions
    :param z_sign: z sign, used if there are two solutions
    :return: x, y, z
    """
    t.measures.clear()
    P.AnchorDic.clear()

    if len(raw_data) < 3:
        raise ValueError("Not enough data for trilateration")
    if len(anchor_positions) < 3:
        raise ValueError("Not enough anchor positions for trilateration")

    matched_ids = list(set(raw_data.keys()) & set(anchor_positions.keys()))
    if len(matched_ids) < 3:
        raise ValueError(f"Not enough common anchors for trilateration: {matched_ids}")

    for id in matched_ids:
        if raw_data[id] is None:
            continue
        P.add_anchor(id, anchor_positions[id])
        t.add_measure(id, raw_data[id])
    P.solve()

    if len(matched_ids) == 3:
        # If there are only 3 anchors, use the z_sign parameter to choose the solution
        if t.loc.z * z_sign < 0:
            t.loc.z = -t.loc.z

    return t.loc.x, t.loc.y, t.loc.z
