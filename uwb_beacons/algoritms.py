
import math
from typing import Dict, Tuple
import localization as lx


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

def calibrated_quadratic(a, b, c, x):
    return a * x**2 + b * x + c 

def calibrated_cubic(a, b, c, d, x):
    return a * x**3 + b * x**2 + c * x + d


def multilateration(raw_data: dict[int, float], anchor_positions: dict[int, Tuple[float, float, float]], z_sign: int = 0) -> Tuple[float, float, float]:
    """
    Trilateration algorithm
    :param raw_data: raw data from the sensor
    :param anchor_positions: anchor positions
    :param z_sign: z sign, used if there are two solutions
    :return: x, y, z
    """
    P=lx.Project(mode='3D',solver='LSE')
    non_zero_data = {}
    for id in raw_data.keys():
        if raw_data[id] is None:
            continue
        non_zero_data[id] = raw_data[id]

    if len(non_zero_data) < 3:
        raise ValueError(f"Not enough data for trilateration non zero:{non_zero_data} input:{raw_data}")
    if len(anchor_positions) < 3:
        raise ValueError(f"Not enough anchor positions for trilateration {anchor_positions}")

    matched_ids = list(set(non_zero_data.keys()) & set(anchor_positions.keys()))
    if len(matched_ids) < 3:
        raise ValueError(f"Not enough common anchors for trilateration {matched_ids}")

    for id in matched_ids:
        P.add_anchor(id, anchor_positions[id])
    t, label = P.add_target()
    for id in matched_ids:
        t.add_measure(id, non_zero_data[id])
    for measure in t.measures:
        print(f"Measure {measure}")
    for anchor in P.AnchorDic:
        print(f"Anchor {anchor}: {P.AnchorDic[anchor]}")
    P.solve()
    if (z_sign != 0):
        t.loc.z = abs(t.loc.z) * z_sign
    return t.loc.x, t.loc.y, t.loc.z
