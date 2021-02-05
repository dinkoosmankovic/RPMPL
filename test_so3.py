from state_spaces.so3 import SO3
from state_spaces.compound_space import CompoundSpace


so3 = SO3()
q1 = so3.get_qrand()
print("q1: ", q1)
print(q1*q1.conjugate)
print(q1.x)
print(q1.norm)

q2 = so3.get_qrand()
print("q2: ", q2)

q_new = so3.get_qnew(q1, q2, 1.5)
print("q_new:" , q_new)
print(so3.distance(q1, q_new))


cs = CompoundSpace([SO3(), SO3(), SO3()])
cs_qrand = cs.get_qrand()
print(cs_qrand)