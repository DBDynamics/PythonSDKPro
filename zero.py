from DBDynamicsPro import Bee

m = Bee()

m.setHomingDirection(0, -1)
m.setHomingDirection(1, 1)
m.setHomingDirection(2, -1)
m.setHomingLevel(0, 0)
m.setHomingLevel(1, 0)
m.setHomingLevel(2, 0)
m.setHomingMode(0)
m.setHomingMode(1)
m.setHomingMode(2)


print('home done')
m.stop()