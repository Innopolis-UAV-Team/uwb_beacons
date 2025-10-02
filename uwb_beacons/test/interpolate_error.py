import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# читаем файл
df = pd.read_csv("results0.txt", sep="\t")

mean_m = df["mean_m"].values
mean_error_m = df["mean_error_m"].values

plt.figure(figsize=(9,6))
plt.scatter(mean_m, mean_error_m, label="Данные", alpha=0.7)

# 1. Линейная аппроксимация
K, B = np.polyfit(mean_m, mean_error_m, 1)
y_lin = K * mean_m + B
r2_lin = 1 - np.sum((mean_error_m - y_lin)**2) / np.sum((mean_error_m - np.mean(mean_error_m))**2)
plt.plot(mean_m, y_lin, "r-", label=f"Линейная: y={K:.4f}x+{B:.4f}, R²={r2_lin:.4f}")

# 2. Квадратичная аппроксимация
coef_quad = np.polyfit(mean_m, mean_error_m, 2)
y_quad = np.polyval(coef_quad, mean_m)
r2_quad = 1 - np.sum((mean_error_m - y_quad)**2) / np.sum((mean_error_m - np.mean(mean_error_m))**2)
plt.plot(mean_m, y_quad, "g--", label=f"Квадрат: y={coef_quad[0]:.4f}x²+{coef_quad[1]:.4f}x+{coef_quad[2]:.4f}, R²={r2_quad:.4f}")

# 3. Кубическая аппроксимация (если данных много — можно добавить)
coef_cubic = np.polyfit(mean_m, mean_error_m, 3)
y_cubic = np.polyval(coef_cubic, mean_m)
r2_cubic = 1 - np.sum((mean_error_m - y_cubic)**2) / np.sum((mean_error_m - np.mean(mean_error_m))**2)
plt.plot(mean_m, y_cubic, "b-.", label=f"Кубическая: R²={r2_cubic:.4f}")

plt.xlabel("Среднее измеренное значение, м")
plt.ylabel("Средняя ошибка, м")
plt.title("Средняя ошибка датчика vs Среднее измерение")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

print(f"[Линейная] K={K:.6f}, B={B:.6f}, R²={r2_lin:.4f}")
print(f"[Квадратичная] coef={coef_quad}, R²={r2_quad:.4f}")
print(f"[Кубическая] coef={coef_cubic}, R²={r2_cubic:.4f}")
