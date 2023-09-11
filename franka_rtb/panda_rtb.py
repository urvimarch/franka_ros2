import roboticstoolbox as rtb

#panda = rtb.models.DH.Panda()

#print(panda)

#T = panda.fkine(panda.qz)

#print(T)

panda = rtb.models.URDF.Panda()

print(panda)
T = panda.fkine(panda.qz, end='panda_hand')
#panda.plot(panda.qz, backend="swift")
traj = rtb.jtraj(panda.qz, panda.qr, 100)
print(traj)
