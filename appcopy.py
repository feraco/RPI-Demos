import os
import json

# Define the license block (HTML <small> for smaller font)
license_cell = {
    "cell_type": "markdown",
    "metadata": {},
    "source": [
        "<small>\n",
        "Part of the InnovatED STEM and DroneBlocks Land, Air, and Sea Robotics Curriculum  \n",
        "Licensed for educational use in schools only.  \n",
        "Redistribution, commercial use, or resale is strictly prohibited.  \n",
        "© 2025 InnovatED STEM & DroneBlocks. All rights reserved.\n",
        "</small>"
    ]
}

# Set the base directory (change this if needed)
base_dir = "."

# Walk through all subdirectories and files
for root, dirs, files in os.walk(base_dir):
    for file in files:
        if file.endswith(".ipynb"):
            path = os.path.join(root, file)

            with open(path, "r", encoding="utf-8") as f:
                try:
                    notebook = json.load(f)
                except json.JSONDecodeError:
                    print(f"❌ Skipping unreadable file: {path}")
                    continue

            # Check if license already exists
            if notebook["cells"] and "DroneBlocks" in "".join(notebook["cells"][0].get("source", [])):
                print(f"✅ License already exists in: {path}")
                continue

            # Add license cell to the top
            notebook["cells"].insert(0, license_cell)

            with open(path, "w", encoding="utf-8") as f:
                json.dump(notebook, f, indent=1)

            print(f"✅ License added to: {path}")

