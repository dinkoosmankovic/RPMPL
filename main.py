import time
import csv
import tests.rrt_test as rt
import tests.drrt_test as drt
import tests.ekf_rrt_test as ekf
import logging

# rt.run(animate=True, choice=0)

a, b, c, d = [], [], [], []
for i in range(0, 100):
    try:
        logging.info("easy - rrt")
        replan_counter, total_time, initial_planning, counter = \
            rt.run(run_number=i, animate=False, choice=0)
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
    except ValueError:
        continue
with open('results_new/rrt_2dof_easy.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
for i in range(0, 100):
    try:
        logging.info("hard - rrt")
        replan_counter, total_time, initial_planning, counter = \
            rt.run(run_number=i, animate=False, choice=1)
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
    except ValueError:
        continue
with open('results_new/rrt_2dof_hard.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
for i in range(0, 100):
    try:
        logging.info("easy - drrt")
        replan_counter, total_time, initial_planning, counter = \
            drt.run(run_number=i, animate=False, choice=0)
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
    except ValueError:
        continue
with open('results_new/drrt_2dof_easy.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
for i in range(0, 100):
    try:
        logging.info("hard - drrt")
        replan_counter, total_time, initial_planning, counter = \
            drt.run(run_number=i, animate=False, choice=1)
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
    except ValueError:
        continue
with open('results_new/drrt_2dof_hard.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
for i in range(0, 100):
    try:
        logging.info("easy - ekf")
        replan_counter, total_time, initial_planning, counter = \
            ekf.run(run_number=i, animate=False, choice=0)
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
    except ValueError:
        continue
with open('results_new/ekf_2dof_easy.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
for i in range(0, 100):
    try:
        logging.info("hard - ekf")
        replan_counter, total_time, initial_planning, counter = \
            ekf.run(run_number=i, animate=False, choice=1)
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
    except ValueError:
        continue
with open('results_new/ekf_2dof_hard.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])
