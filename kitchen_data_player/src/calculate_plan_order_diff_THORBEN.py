#!/usr/bin/env python
import csv
import math
import sys
import time
import difflib


# Usage: calculate_plan_confidence_value.py symbolic_plan.csv

planReader = csv.DictReader(open(sys.argv[1], 'rb'), delimiter=',', quotechar='|')

model_plans = {'DW': 'GKIK', 'STC': 'AKDKCKBK', 'STQ': 'DKDKCKBK', 'CTC': 'AKDKEKEKEK', 'CTQ': 'DKDKEKEKEK', 'PW': 'KHJ'}
plan = ''
threshhold = 0.005

def levenshtein(s1, s2):
    if len(s1) < len(s2):
        return levenshtein(s2, s1)
    if not s1:
        return len(s2)
 
    previous_row = xrange(len(s2) + 1)
    for i, c1 in enumerate(s1):
        current_row = [i + 1]
        for j, c2 in enumerate(s2):
            insertions = previous_row[j + 1] + 1 # j+1 instead of j since previous_row and current_row are one character longer
            deletions = current_row[j] + 1       # than s2
            substitutions = previous_row[j] + (c1 != c2)
            current_row.append(min(insertions, deletions, substitutions))
        previous_row = current_row
 
    return previous_row[-1]

for row in planReader:
    # Write new csv-file with: instance, time, BECX, BEXY, BECTheta, 
   
    if (row['location'] == 'TABLE' and float(row['probability']) > threshhold ):
        plan += 'K'
         
    if (row['location'] == 'OVEN' and float(row['probability']) > threshhold):
        plan += 'A'
       
    if (row['location'] == 'DRAWER' and float(row['probability']) > threshhold ):
        plan += 'B'
     
    if (row['location'] == 'CUPBOARD0' and float(row['probability']) > threshhold ):
        plan += 'C'

    if (row['location'] == 'REFRIGERATOR' and float(row['probability']) > threshhold ):
        plan += 'D'    
        
    if (row['location'] == 'SINK' and float(row['probability']) > threshhold ):
        plan += 'E'     

    if (row['location'] == 'CUPBOARD0' and float(row['probability']) > threshhold ):
        plan += 'F'

    if (row['location'] == 'BOTTLE-PLACE' and float(row['probability']) > threshhold ):
        plan += 'G'

    if (row['location'] == 'TABLE2' and float(row['probability']) > threshhold ):
        plan += 'H'

    if (row['location'] == 'CUPBOARD1' and float(row['probability']) > threshhold ):
        plan += 'I'

    if (row['location'] == 'DOOR' and float(row['probability']) > threshhold ):
        plan += 'J'

#print('Comparing plans: [ %s ] with model [ %s ]'%(plan, model_plan))


#print('_______________________________________')
#print('Similarity: %15s            ([ %s ]  %s --> plan %s)'%(seq.ratio(), sys.argv[1], model_plan, plan))

# max levenshtein_dist = 14 for our model

for key in model_plans:
    levenshtein_distance = levenshtein(plan, model_plans[key])
    seq=difflib.SequenceMatcher(a=plan.lower(), b=model_plans[key].lower())
    seq.ratio()
    conf_value = (14 - float(levenshtein_distance)) / 14
    print('GLS for %3s: %15s  (LD: %2s)          ([ %s ]  %s --> plan %s)'%(key, conf_value, levenshtein_distance, sys.argv[1], model_plans[key], plan))


        

