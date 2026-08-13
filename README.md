# TIMDR Radar Module

Moduł analizy trajektorii ruchu (`timdr_radar.py`): gradient przepływu
(TIMDR-flow), detekcja topologicznych "twistów" (nagłych zmian kierunku),
redukcja szumu trajektorii (TRM) i prosta predykcja ruchu.

## Status

Kod ze zgłoszenia uruchomiony i przetestowany (`test_timdr_radar.py`,
10/10 testów przechodzi). Znaleziono i naprawiono jeden realny błąd
matematyczny oraz kilka braków w walidacji wejścia.

![Bug zawijania kąta w detektorze twist](screenshot_twist_bugfix.png)

### 🐛 Błąd: zawijanie kąta w `twist()`

`arctan2` zwraca kierunek ruchu w zakresie `(-π, π]`. Gdy trajektoria
porusza się w kierunku bliskim ±180°, kolejne kąty potrafią przeskakiwać
z ok. `+177°` na ok. `-178°` — fizycznie to zmiana kierunku o ~5°, ale
naiwna różnica liczbowa dwóch kątów daje ~355° (prawie pełny obrót).

Zweryfikowano na trajektorii oscylującej wokół kierunku ~180° z
rzeczywistą zmianą kierunku ~1–6° na krok:

- **oryginalny kod** (`np.gradient(angles)` bez obsługi zawijania):
  zgłaszał różnice kąta ±355°, ±179°, ±177° → **4 fałszywe alarmy
  twist na 5 punktów**, mimo że trajektoria była niemal prostoliniowa.
- **zawinięcie różnicy *po* gradiencie** (`(dtheta+π) % 2π - π`) —
  intuicyjna pierwsza poprawka — **nadal błędne** dla różnic centralnych
  (jakich używa `np.gradient` w punktach wewnętrznych): 2 z 5 punktów
  nadal dawały ~179°.
- **poprawka właściwa**: `np.unwrap()` na całym ciągu kątów *przed*
  różniczkowaniem (dodaje/odejmuje wielokrotności 2π tak, by sąsiednie
  wartości nigdy nie różniły się o więcej niż π) → wszystkie 5 punktów
  daje poprawne różnice ~1–6°, **0 fałszywych alarmów**.

Test regresyjny: `test_zawijanie_kata_nie_daje_falszywego_alarmu`.

### Pozostałe poprawki (walidacja wejścia)

- **`_validate()`**: sprawdza kształt `(N, 3)`, minimalną liczbę punktów
  i to, że znaczniki czasu są ściśle rosnące. Bez tego `dt <= 0`
  (powtórzony lub cofnięty czas) powodował dzielenie przez zero w
  `np.gradient(pos, t, axis=0)` → `inf`/`NaN` propagujące się przez
  wszystkie dalsze obliczenia bez żadnego ostrzeżenia.
- **`timdr_flow()`**: ostatni gradient (`v + a`) teraz też liczony
  względem rzeczywistego czasu `t`, tak jak `v` i `a` — oryginalny kod
  mieszał gradient "po czasie" z gradientem "po indeksie próbki", co
  dawało niespójny wynik przy nierównomiernym próbkowaniu.

### Przykład użycia (identyczny jak w zgłoszeniu)

```python
from timdr_radar import TIMDRRadar

radar = TIMDRRadar()
traj = [[0,0,0],[1,0.2,1],[2,0.5,2],[3,1.2,3],[4,2.5,4]]

flow = radar.timdr_flow(traj)
twist = radar.twist(traj)
stable = radar.trm_reduce(traj)
pred = radar.predict(traj)
```

Uruchomienie: `python demo.py` / testy: `pytest -q`.
