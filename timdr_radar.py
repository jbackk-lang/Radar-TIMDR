"""
TIMDR Radar Module
===================
Moduł analizy trajektorii ruchu: gradient przepływu (TIMDR-flow),
detekcja topologicznych "twistów" (nagłych zmian kierunku),
redukcja szumu (TRM) i prosta predykcja ruchu.

Wejście: trajektoria jako lista/tablica punktów [x, y, t].
"""

import numpy as np


class TIMDRRadar:
    def __init__(self):
        pass

    # ------------------------------------------------------------
    # walidacja wspólna dla wszystkich metod
    # ------------------------------------------------------------
    @staticmethod
    def _validate(trajectory, min_points=2):
        traj = np.asarray(trajectory, dtype=np.float64)
        if traj.ndim != 2 or traj.shape[1] != 3:
            raise ValueError(
                f"trajectory musi mieć kształt (N, 3) [x, y, t], dostano {traj.shape}"
            )
        if len(traj) < min_points:
            raise ValueError(
                f"trajectory musi mieć co najmniej {min_points} punkty, dostano {len(traj)}"
            )
        t = traj[:, 2]
        if np.any(np.diff(t) <= 0):
            # POPRAWKA: np.gradient(pos, t, axis=0) dzieli przez różnice czasu.
            # Nierosnący lub powtórzony znacznik czasu (dt <= 0) daje dzielenie
            # przez zero -> inf/NaN w v, a i we wszystkich dalszych obliczeniach.
            raise ValueError(
                "znaczniki czasu (kolumna t) muszą być ściśle rosnące (dt > 0 między "
                "kolejnymi punktami) - inaczej gradient dzieli przez zero"
            )
        return traj

    @staticmethod
    def _wrap_angle_diff(dtheta):
        # Pomocnicze: normalizacja pojedynczej roznicy kata do (-pi, pi].
        # Uzywane w testach jednostkowych jako sanity-check; wlasciwa
        # poprawka bugu zawijania jest w unwrap_angles() ponizej.
        return (dtheta + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def _unwrapped_angles(v):
        return np.unwrap(np.arctan2(v[:, 1], v[:, 0]))

    # --- 1. TIMDR-flow: gradient ruchu ---
    def timdr_flow(self, trajectory):
        """
        trajectory: lista punktów [[x,y,t], ...]
        zwraca: gradient ruchu (TIMDR-flow) = d(v+a)/dt
        """
        traj = self._validate(trajectory)
        pos = traj[:, :2]
        t = traj[:, 2]

        v = np.gradient(pos, t, axis=0)
        a = np.gradient(v, t, axis=0)

        # POPRAWKA: oryginalny kod liczył np.gradient(v + a, axis=0) bez
        # przekazania `t`, czyli traktował próbki jako równoodległe w czasie
        # (spacing=1), mimo że v i a były liczone z prawdziwym dt. Dla
        # nierównomiernie próbkowanej trajektorii dawało to niespójną,
        # zniekształconą wartość "flow". Ujednolicono - flow też liczone
        # względem rzeczywistego czasu t.
        flow = np.gradient(v + a, t, axis=0)
        return flow

    # --- 2. Twist detector: zmiana kierunku ---
    def twist(self, trajectory, threshold=0.35):
        """
        Wykrywa topologiczny twist (nagłą zmianę kierunku ruchu).
        threshold: próg w radianach (domyślnie 0.35 rad ~= 20 stopni)
        """
        traj = self._validate(trajectory)
        pos = traj[:, :2]
        t = traj[:, 2]

        v = np.gradient(pos, t, axis=0)

        # POPRAWKA (bug zawijania kąta / angle wrap-around):
        # Kierunek "tuż pod +pi" i "tuż nad -pi" to fizycznie prawie ten sam
        # kierunek (różnica kilku stopni), ale naiwna różnica kolejnych
        # wartości arctan2 (zakres (-pi, pi]) daje skok ~2*pi (~360 stopni)
        # i generuje fałszywy alarm "twist" niemal w każdym punkcie, gdy
        # trajektoria oscyluje wokół kierunku ~180 stopni.
        #
        # Samo "zawinięcie różnicy po fakcie" (modulo 2*pi na wyniku
        # gradientu) NIE wystarcza dla różnic centralnych (używanych przez
        # np.gradient w punktach wewnętrznych) - potwierdzone empirycznie:
        # dawało to nadal błędne wartości ~179 stopni zamiast realnych ~1-3
        # stopni w dwóch z pięciu punktów testowych. Poprawny sposób to
        # np.unwrap() na CIĄGU kątów przed różniczkowaniem (dodaje/odejmuje
        # wielokrotności 2*pi tak, by sąsiednie wartości nigdy nie różniły
        # się o więcej niż pi), a dopiero potem np.gradient.
        angles_unwrapped = self._unwrapped_angles(v)
        dtheta = np.gradient(angles_unwrapped)

        twist_points = np.where(np.abs(dtheta) > threshold)[0]
        return twist_points

    # --- 3. TRM-reduction: stabilizacja trajektorii ---
    def trm_reduce(self, trajectory):
        """
        TRM: prosta redukcja szumu (średnia krocząca 3-punktowa).
        Uwaga: pierwszy i ostatni punkt pozostają bez zmian (brak
        sąsiadów z obu stron) - to świadome uproszczenie, nie błąd,
        ale warto o tym wiedzieć przy krótkich trajektoriach.
        """
        traj = self._validate(trajectory)
        pos = traj[:, :2]

        smooth = pos.copy()
        for i in range(1, len(pos) - 1):
            smooth[i] = (pos[i - 1] + pos[i] + pos[i + 1]) / 3.0

        return smooth

    # --- 4. Predykcja ruchu ---
    def predict(self, trajectory, steps=5):
        """
        Predykcja pozycji na `steps` kroków w przód na podstawie
        ostatniej znanej prędkości i przyspieszenia (ruch jednostajnie
        przyspieszony ekstrapolowany ze stałym dt = ostatni krok czasowy).
        """
        traj = self._validate(trajectory)
        pos = traj[:, :2]
        t = traj[:, 2]

        v = np.gradient(pos, t, axis=0)
        a = np.gradient(v, t, axis=0)

        dt = t[-1] - t[-2]  # bezpieczne: _validate gwarantuje dt > 0

        pred = []
        p = pos[-1].copy()
        v0 = v[-1].copy()
        a0 = a[-1].copy()

        for _ in range(steps):
            v0 = v0 + a0 * dt
            p = p + v0 * dt
            pred.append(p.copy())

        return np.array(pred)
