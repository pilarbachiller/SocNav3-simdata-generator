import os
import sys
import json
import copy
from sklearn import metrics as sk
import pandas as pd
import seaborn as sns
import pingouin as pg

import numpy as np
import matplotlib.pyplot as plt

from collections import namedtuple

Response = namedtuple('Response', ['id', 'total_scores', 'keys', 'values', 'rep_keys', 'rep_values', 'rep2_keys', 'rep2_values'])

JSON_DIR = '.'

MIN_RELEVANT = 7
RELEVANT = [
    [7,  3007,   "A robot is trying to locate the source of a noise in a library."],
    [11, 2007,   "A robot is navigating as part of a delivery task in a museum."],
    [13, 1007,   "An office assistant robot keeps track of who is in the office today."],
    [17, 7,      "A hotel robot is inspecting the floor to ensure it's safe to walk."],
    [19, 302,    "A drug delivery robot is working in a hospital."],
    [23, 1302,   "A museum robot roams around looking for people interested in its services."],
    [31, 2302, "A robot is performing routine tasks in an office."],
    [53, 3102,   "A museum guide robot has been asked to go to the goal shown, with no additional context."],
    [59, 2002,   "A lab assistant robot is looking for potential hazards in its environment."],
    [61, 1002,   "A cleaning robot working in a hospital is looking for dirty spots to clean."],
    [67, 2,      "The robot is trying to locate the glasses of a patient in a hospital."],
    [71, 3094,   "A hospital assistant robot has been asked to go to the goal, with no additional context."],
    [73, 2894,   "An idle robot working in a museum goes to recharge its battery. It has 13% battery left."],
    [79, 1879,   "A assistant robot is performing routine tasks in a restaurant."],
    [83, 834,    "A warehouse robot is moving around while inspecting the air quality."],
    ]

MIN_REPS = 5
REPS = [
    [29, 7],
    [37, 11],
    [41, 13],
    [43, 17],
    [47, 19],
]


def is_sorted(lst):
    return lst == sorted(lst)

def firsts(t):
    return [e[0] for e in t]

def trajectory_id(k):
    for r in RELEVANT:
        if r[0]==k:
            return r[1]
    return -1

def trajectory_description(k):
    for r in RELEVANT:
        if r[0]==k:
            return r[2]
    return "none"

def rep_id(k):
    for r in REPS:
        if r[0]==k:
            return r[1]
    return -1

def get_responses():
    total_surveys = 0
    total_answers = 0

    responses = []

    # Iterate over files in the directory
    for filename in os.listdir(JSON_DIR):
        if filename.endswith('.json'):
            file_path = os.path.join(JSON_DIR, filename)
            with open(file_path, 'r') as f:
                keys = []
                values = []
                rep_keys = []
                rep_values = []
                rep2_keys = []
                rep2_values = []                
                try:
                    text = ''.join(f.readlines())
                    text = text.replace("\n", "")
                    data = json.loads(text)
                    for k, v in data["answers"].items():
                        if int(k) in firsts(RELEVANT):
                            keys.append(int(k))
                            values.append(int(v*100))
                            if data["indices"][int(k)] != trajectory_id((int(k))):
                                print("WRONG INDEX!!!", data["indices"][int(k)], trajectory_id((int(k))), file_path)
                            if data["descriptions"][int(k)] != trajectory_description((int(k))):
                                print("WRONG DESCRIPTION!!!", data["descriptions"][int(k)], trajectory_description((int(k))), file_path)

                        if int(k) in firsts(REPS):
                            k2 = rep_id(int(k))
                            if str(k2) in data["answers"].keys():
                                rep_keys.append(int(k))
                                rep_values.append(int(v*100))
                                rep2_keys.append(k2)
                                rep2_values.append(int(data["answers"][str(k2)]*100))
                                if data["indices"][int(k)] != data["indices"][int(k2)]:
                                    print("WRONG INDEX IN REP DATA!!!", data["indices"][int(k)], data["indices"][int(k2)], file_path)


                    r = Response(id=file_path, total_scores=len(data["answers"]), keys=keys, values=values, rep_keys=rep_keys, rep_values=rep_values, rep2_keys=rep2_keys, rep2_values=rep2_values)
                    responses.append(r)
                    total_surveys += 1
                    total_answers += len(data["answers"])
                except json.JSONDecodeError:
                    print(f"Error decoding JSON in file: {filename}")

    # Now json_data contains the loaded data from all JSON files
    print(f"We have {total_surveys} surveys, containing {total_answers} answers.")
    return responses


def filter_out_invalid_responses(responses):
    good_responses = []
    keys = None
    for r in responses:

        if len(r.keys)<MIN_RELEVANT:
            continue
            print("Didn't answer all control.")
        if len(r.rep_keys)<MIN_REPS:
            continue
            print("Didn't answer all rep control.")
        
        if is_sorted(r.keys) is False:
            print("KEYS ARE NOT SORTED???")
            sys.exit(-1)

        if is_sorted(r.rep_keys) is False:
            print("REP EYS ARE NOT SORTED???")

        if keys is None:
            print(f"Setting keys: {r.keys[:MIN_RELEVANT]}")
            keys = r.keys[:MIN_RELEVANT]
        if r.keys[:MIN_RELEVANT] != keys[:MIN_RELEVANT]:
            print(f"Key lists do not coincide: \n{r.keys[:MIN_RELEVANT]}\n{keys[:MIN_RELEVANT]}")
            sys.exit(-1)

        good_responses.append(r)
    return good_responses

def sort_x_like_y(xv, yv):
    x = copy.deepcopy(xv)
    y = copy.deepcopy(yv)
    sy, sx = zip(*sorted(zip(y, x)))
    sx = list(sx)
    sy = list(sy)
    return sx, sy

def sort_x_with_indices(xv, indices):
    ret = [-1 for _ in indices]
    for index in indices:
        ret[indices[index]] = xv[indices[index]]
    return ret

def get_values_from_responses(responses):
    matrix = np.zeros((len(responses), MIN_RELEVANT))
    print(matrix.shape)
    for response_idx, response in enumerate(responses):
        matrix[response_idx,:] = response.values[:MIN_RELEVANT]
    return matrix

def get_total_number_of_scores(responses):
    t = 0
    for response in responses:
        t += response.total_scores
    return t

def get_consistency_matrix(responses):
    matrix = np.ndarray((len(responses), len(responses)), dtype=float)
    for i1, p1 in enumerate(responses):
        for i2, p2 in enumerate(responses):
            if p1.id == p2.id:
                val1 = p1.rep_values[:MIN_REPS]
                val2 = p1.rep2_values[:MIN_REPS]
            else:
                val1 = p1.values[:MIN_RELEVANT]
                val2 = p2.values[:MIN_RELEVANT]
            matrix[i1, i2] = sk.cohen_kappa_score(val2, val1, labels=list(range(101)), weights='quadratic')
    return matrix

def get_icc(responses):
    participants = ['p'+str(p) for p in range(len(responses))]
    data = {'Participant': participants}
    keys = responses[0].keys[:MIN_RELEVANT]
    for i, k in enumerate(keys):
        scores = [r.values[i] for r in responses]
        data['Q'+str(k)] = scores

    df_icc = pd.DataFrame(data)
    df_long = df_icc.melt(id_vars=['Participant'], var_name='Question', value_name='Rating')
    icc_result = pg.intraclass_corr(data=df_long, targets='Question', raters='Participant', ratings='Rating')
    print(icc_result)

if __name__ == "__main__":
    print("Responses length:", MIN_RELEVANT)
    total_responses = get_responses()
    print(f"We have {len(total_responses)} responses.")
    valid_responses = filter_out_invalid_responses(total_responses)
    print(f"We have {len(valid_responses)} valid responses.")

    consistency_matrix = get_consistency_matrix(valid_responses)
    raters = ['p'+str(i) for i in range(len(valid_responses))]

    df = pd.DataFrame(consistency_matrix, index=raters, columns=raters)
    plt.figure(figsize=(8, 6))
    sns.heatmap(df, annot=True, cmap="RdYlGn", cbar=True, square=True, vmin=-1, vmax=1)
    plt.title("Raters' consistency")
    plt.show()



    tscores_before = get_total_number_of_scores(valid_responses)

    valid_responses_after_inconsistency_test = []
    for p, r in enumerate(valid_responses):
        if consistency_matrix[p,p] > 0.4:
            valid_responses_after_inconsistency_test.append(r)
        else:
            print(f'{r.id} ({raters[p]}) removed from valid responses')

    tscores_after = get_total_number_of_scores(valid_responses_after_inconsistency_test)

    get_icc(valid_responses_after_inconsistency_test)

    print(f'Initial and final number of scores: {tscores_before} {tscores_after}' )

    valid_responses = valid_responses_after_inconsistency_test
    values = get_values_from_responses(valid_responses)
    averages = np.mean(values, axis=0)
    # print(f"{values=}")
    # print(f"{averages}")

    sorted_indices, sorted_averages = sort_x_like_y([x for x in range(MIN_RELEVANT)], averages)
    sorted_questions, sorted_averages = sort_x_like_y(valid_responses[0].keys, averages)

    plt.figure(figsize=(10,5))
    plt.plot(sorted_averages, linewidth=6, label='average')

    for response in valid_responses:
        sorted_values, _ = sort_x_like_y(response.values[:MIN_RELEVANT], averages)
        print(["%.2f" % a for a in sorted_values[:MIN_RELEVANT]])
        plt.plot(sorted_values, linestyle="-.")
    plt.legend()
    ticks = [x for x in range(MIN_RELEVANT)]
    labels = [f"Q{l}" for l in sorted_questions]

    print(labels)
    print(ticks)
    plt.xticks(ticks=ticks, labels=labels)

    plt.show()

