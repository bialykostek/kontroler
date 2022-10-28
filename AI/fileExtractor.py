file = open('log_27_10_01_24_53.txt', 'r')
lines = [[0, 2], [4, 10]]

lineId = 0
output = []

while True:
    line = file.readline()
  
    if not line:
        break

    line = line.split(",")
    