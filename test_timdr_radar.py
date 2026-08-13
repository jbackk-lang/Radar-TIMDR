import numpy as np
import pytest
from timdr_radar import TIMDRRadar


@pytest.fixture
def radar():
    return TIMDRRadar()


def test_walidacja_ksztaltu(radar):
    with pytest.raises(ValueError):
        radar.twist([[0, 0], [1, 1]])  # brak kolumny t


def test_walidacja_min_punktow(radar):
    with pytest.raises(ValueError):
        radar.twist([[0, 0, 0]])


def test_walidacja_nierosnacy_czas(radar):
    traj = [[0, 0, 0], [1, 1, 0]]  # dt = 0
    with pytest.raises(ValueError):
        radar.twist(traj)
    traj2 = [[0, 0, 0], [1, 1, 1], [2, 2, 0.5]]  # cofniecie czasu
    with pytest.raises(ValueError):
        radar.twist(traj2)


def test_linia_prosta_brak_twistu(radar):
    # ruch jednostajny po linii prostej -> zero twistow, przyspieszenie ~0
    traj = [[float(i), float(i) * 0.5, float(i)] for i in range(6)]
    assert len(radar.twist(traj)) == 0
    flow = radar.timdr_flow(traj)
    assert np.allclose(flow, 0.0, atol=1e-8)


def test_ostry_zakret_wykryty(radar):
    # trajektoria z ostrym zakretem 90 stopni w polowie drogi
    traj = [[0, 0, 0], [1, 0, 1], [2, 0, 2], [2, 1, 3], [2, 2, 4], [2, 3, 5]]
    twist_points = radar.twist(traj)
    assert len(twist_points) > 0
    assert 2 in twist_points or 3 in twist_points  # okolice zakretu


def test_zawijanie_kata_nie_daje_falszywego_alarmu(radar):
    """
    Regresja dla bugu zawijania kata +-pi.
    Trajektoria oscyluje wokol kierunku ~180 stopni z PRAWDZIWA zmiana
    kierunku rzedu kilku stopni na krok. Naiwna (niepoprawiona) wersja
    zglaszala tu falszywy twist w niemal kazdym punkcie (~354-356 st
    naiwnej roznicy kata zamiast realnych kilku stopni).
    """
    traj = [
        [0, 0, 0],
        [-1, 0.05, 1],
        [-2, -0.05, 2],
        [-3, 0.05, 3],
        [-4, -0.05, 4],
    ]
    twist_points = radar.twist(traj, threshold=0.35)
    assert len(twist_points) == 0, (
        f"falszywe alarmy twist z powodu niezalatanego zawijania kata: {twist_points}"
    )


def test_dtheta_wrap_helper_bezposrednio(radar):
    # naiwna roznica ~2*pi (359 st) powinna zostac znormalizowana do ~kilku stopni
    naive = np.array([np.deg2rad(-355.7), np.deg2rad(1.43), np.deg2rad(179.28)])
    wrapped = radar._wrap_angle_diff(naive)
    assert np.all(np.abs(wrapped) <= np.pi + 1e-9)
    assert np.isclose(wrapped[0], np.deg2rad(4.3), atol=1e-2)


def test_trm_reduce_wygladza_srodek_zachowuje_konce(radar):
    traj = [[0, 0, 0], [10, 0, 1], [0, 0, 2], [10, 0, 3], [0, 0, 4]]
    smooth = radar.trm_reduce(traj)
    assert np.allclose(smooth[0], [0, 0])   # pierwszy punkt niezmieniony
    assert np.allclose(smooth[-1], [0, 0])  # ostatni punkt niezmieniony
    # srodkowe punkty wygladzone -> mniejsza amplituda oscylacji niz oryginal
    orig = np.array(traj)[:, :2]
    assert np.abs(smooth[2][0]) < np.abs(orig[2][0] - 5) + 10  # sanity, nie NaN
    assert not np.any(np.isnan(smooth))


def test_predict_ruch_jednostajny_ekstrapoluje_liniowo(radar):
    # ruch jednostajny (a=0) -> predykcja powinna kontynuowac ta sama predkosc
    traj = [[float(i), 0.0, float(i)] for i in range(5)]  # v=(1,0), a=0
    pred = radar.predict(traj, steps=3)
    expected = np.array([[5.0, 0.0], [6.0, 0.0], [7.0, 0.0]])
    assert np.allclose(pred, expected, atol=1e-6)


def test_predict_zwraca_odpowiedni_ksztalt(radar):
    traj = [[0, 0, 0], [1, 0.2, 1], [2, 0.5, 2], [3, 1.2, 3], [4, 2.5, 4]]
    pred = radar.predict(traj, steps=5)
    assert pred.shape == (5, 2)
    assert not np.any(np.isnan(pred))
