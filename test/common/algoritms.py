
import math
from typing import Tuple


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
