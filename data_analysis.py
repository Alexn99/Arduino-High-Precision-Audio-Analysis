import csv

data = list()

with open('gps_drift_digital.csv', newline='') as f:
  reader = csv.reader(f, delimiter=',')
  for row in reader:
      data.append(row[0])
#print(data)

i = 0

f = open('gps_drift_digital_processed.csv')
writer = csv.writer(f, delimiter=',')

for pt in data:
    if int(pt) > i*1000000:
        print(float(pt)/1000000)
        i += 1

f.close()
