import re
import pandas as pd

with open("odom_raw.txt", "r") as f:
    lines = f.readlines()

data = []
entry = {}

for line in lines:
    line = line.strip()

    if line == "---":
        if entry:
            data.append(entry)
            entry = {}
        continue

    if "position:" in line:
        current_section = "position"
    elif "orientation:" in line:
        current_section = "orientation"
    elif "twist:" in line:
        current_section = "twist"

    # Match lines like "x: 1.23"
    match = re.match(r"^(x|y|z|w):\s*(-?\d+\.\d+)", line)
    if match:
        key, value = match.groups()
        full_key = f"{current_section}_{key}"
        entry[full_key] = float(value)

# Add last message if any
if entry:
    data.append(entry)

# Convert to DataFrame
df = pd.DataFrame(data)

# Save to CSV
df.to_csv("odom_data.csv", index=False)
print("✅ odom_data.csv created with", len(df), "rows.")
