# Resultados Bayesian Optimization — Clásico

Fecha: 2026-05-05

---

## Motor DC — `motor_bayes_opt_2.m`

| Parámetro | Valor |
|-----------|-------|
| Kp | 2.3480 |
| Ki | 9.9974 |
| Kd | 0.2064 |
| ITAE BO | 0.0660 |
| ITAE baseline (Kp=25, Ki=2.5, Kd=1.5) | 7.0358 |
| Mejora | 99.1% |

**Notas:**
- Ki llegó al límite máximo del rango de búsqueda (10.0) — considerar ampliar rango en iteraciones futuras.
- 25 evaluaciones, 6.9s total.

---

## Hotend — `fred_clasico_bo_hotend.m`

| Parámetro | Valor |
|-----------|-------|
| Kp | 9.9657 |
| Ki | 4.9947 |
| Kd | 0.0183 |
| ITAE BO | 2,110,696.9 |

**Notas:**
- ITAE alto es normal — la simulación del hotend corre 800s, el error se acumula por la dinámica lenta (τ≈100s).
- Kd ≈ 0 tiene sentido para un sistema térmico lento.
- 60 evaluaciones, 20.2s total.
- No se tiene ITAE baseline para comparar directamente (script no lo calcula).

---

## Siguiente paso — Hans (Simulink)

Con estos valores, Hans crea dos Simulink (`motor_Gs.mat` y `hotend_Gs.mat`) comparando:
1. **pidtune() baseline** — ganancias calculadas automáticamente por MATLAB
2. **BO optimizado** — valores de esta tabla

Motor: simular 30s | Hotend: simular 800-1000s
