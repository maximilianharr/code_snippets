# This python script creates a test table to test sql performance
import csv
import random
with open('/lhome/harrmax/workspace/daimler_harr_workspace/sql/test.csv', 'w', newline='\n') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for i in range(1,2000000):
        rand_i = random.randint(1,100)
        spamwriter.writerow([ str(i) + ',' + str(rand_i)])
