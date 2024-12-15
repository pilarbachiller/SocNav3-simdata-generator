import os
import json

import seaborn as sns

import matplotlib.pyplot as plt

JSON_DIR = '.'

total_surveys = 0
total_answers = 0

samples = []

values = []
# Iterate over files in the directory
for filename in os.listdir(JSON_DIR):
    if filename.endswith('.json'):
        print(filename)
        file_path = os.path.join(JSON_DIR, filename)
        with open(file_path, 'r') as f:
            try:
                text = ''.join(f.readlines())
                text = text.replace("\n", "")
                data = json.loads(text)
                for k, v in data["answers"].items():
                    samples.append(data["indices"][int(k)])
                    values.append(v)
                total_surveys += 1
                total_answers += len(data["answers"])
            except json.JSONDecodeError:
                print(f"Error decoding JSON in file: {filename}")

# Now json_data contains the loaded data from all JSON files
print(f"We have {total_surveys} surveys, containing {total_answers} answers.")

# print(samples)
bins = max(samples) - min(samples)


plt.figure()
plt.hist(samples, bins=bins, histtype='stepfilled', cumulative=True)
plt.xlabel('Indices')
plt.ylabel('Frequency')
plt.title('Cumulative histogram of the trajectory indices')

plt.figure()
plt.hist(values, bins=100, histtype='stepfilled', cumulative=True)
plt.xlabel('Values')
plt.ylabel('Frequency')
plt.title('Cumulative histogram of the answer\' values')



plt.figure()
sns.histplot(data=samples, kde=True, bins=bins, stat="density", kde_kws={'bw_adjust': 0.1})

plt.xlabel('Value')
plt.ylabel('Density')
plt.title('Histogram with KDE Overlay')

plt.figure()
plt.hist(values, bins=100, histtype='stepfilled', cumulative=False)
plt.xlabel('Indices')
plt.ylabel('Cumulative frequency')
plt.title('Histogram of the answers\' values')

plt.show()

