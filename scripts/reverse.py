import csv
row = []
with open("lines.csv") as fr, open("line2.csv","w", newline='') as fw:
    cr = csv.reader(fr,delimiter=",")
    cw = csv.writer(fw,delimiter=",")
    for line in cr:
        row.append(line)
    row.reverse()
    for r in row:
        cw.writerow([float(r[0]), float(r[1]), float(r[2]),float(r[3]),float(r[4]),float(r[5]), float(r[6])])