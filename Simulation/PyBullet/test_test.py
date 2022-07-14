import csv


# Read in joint trajectories from csv files
q1 = [] # initialize empty list
q2 = [] 
with open('data/q1.csv', newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter='\t')
    for row in reader:
        for element in row: # need two forloops just because of csv file structure
            q1.append(float(element)) # add the numbers to our python list

print(q1)
print(len(q1))