from timdr_radar import TIMDRRadar

radar = TIMDRRadar()

traj = [
    [0, 0, 0],
    [1, 0.2, 1],
    [2, 0.5, 2],
    [3, 1.2, 3],
    [4, 2.5, 4],
]

flow = radar.timdr_flow(traj)
twist = radar.twist(traj)
stable = radar.trm_reduce(traj)
pred = radar.predict(traj)

print("TIMDR-flow:", flow)
print("Twist points:", twist)
print("Stabilized:", stable)
print("Prediction:", pred)
