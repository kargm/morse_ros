#!/usr/bin/env python
import csv
import math
import sys
import time
import difflib
#import jellyfish


# Usage: calculate_plan_confidence_value.py symbolic_plan.csv

planReader = csv.DictReader(open(sys.argv[1], 'rb'), delimiter=',', quotechar='|')

# model-plan for robot table-setting
#model_plan = 'ADADCDBDBDBDCD'
# model-plan for human table-setting
model_plan = 'ADCDBD'
plan = ''
threshhold = 0.000025

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
   
    if (row['location'] == 'table' and float(row['probability']) > threshhold ):
        plan += 'D'
         
    if (row['location'] == 'dishwasher' and float(row['probability']) > threshhold):
        plan += 'A'
       
    if (row['location'] == 'drawer55' and float(row['probability']) > threshhold ):
        plan += 'B'
     
    if (row['location'] == 'drawer115' and float(row['probability']) > threshhold ):
        plan += 'C'
             
#print('Comparing plans: [ %s ] with model [ %s ]'%(plan, model_plan))

seq=difflib.SequenceMatcher(a=plan.lower(), b=model_plan.lower())
seq.ratio()
#print('_______________________________________')
#print('Difflib: %15s            ([ %s ]  %s --> plan %s)'%(seq.ratio(), sys.argv[1], model_plan, plan))

levenshtein_distance = levenshtein(plan, model_plan)
# max levenshtein_dist = 14 for our model
conf_value = (14 - float(levenshtein_distance)) / 14
print('Levenshtein: %15s  (LD: %s)          ([ %s ]  %s --> plan %s)'%(conf_value, levenshtein_distance, sys.argv[1], model_plan, plan))

#print("Levenshtein: %s"%jellyfish.levenshtein_distance(plan, model_plan))
#print("DamerauLevenshtein: %s"%jellyfish.damerau_levenshtein_distance(plan, model_plan))
#print("Jaro: %s"%jellyfish.jaro_distance(plan, model_plan))
#print("Jaro-Winkler: %s"%jellyfish.jaro_winkler(plan, model_plan))       
#print("Hamming: %s"%jellyfish.hamming_distance(plan, model_plan))
