import os
import nbformat
import re

# Emoji regex pattern
emoji_pattern = re.compile("["
    u"\U0001F600-\U0001F64F"  # emoticons
    u"\U0001F300-\U0001F5FF"  # symbols & pictographs
    u"\U0001F680-\U0001F6FF"  # transport & map
    u"\U0001F1E0-\U0001F1FF"  # flags
    u"\U00002500-\U00002BEF"  # misc symbols
    u"\U00002702-\U000027B0"
    u"\U000024C2-\U0001F251"
    u"\U0001f926-\U0001f937"
    u"\U00010000-\U0010ffff"
    u"\u2640-\u2642"
    u"\u2600-\u2B55"
    u"\u200d"
    u"\u23cf"
    u"\u23e9"
    u"\u231a"
    u"\ufe0f"
    u"\u3030"
    "]+", flags=re.UNICODE)

def remove_emojis(text):
    return emoji_pattern.sub('', text)

def clean_notebook_in_place(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        notebook = nbformat.read(f, as_version=4)

    modified = False

    for cell in notebook['cells']:
        if 'source' in cell:
            original = cell['source']
            if isinstance(original, list):
                cleaned = [remove_emojis(line) for line in original]
            else:
                cleaned = remove_emojis(original)

            if original != cleaned:
                cell['source'] = cleaned
                modified = True

    if modified:
        with open(file_path, 'w', encoding='utf-8') as f:
            nbformat.write(notebook, f)
        print(f"✅ Cleaned: {file_path}")
    else:
        print(f"⏭️ No emojis found: {file_path}")

def process_all_notebooks_in_folder(base_directory):
    for root, _, files in os.walk(base_directory):
        for file in files:
            if file.endswith('.ipynb'):
                full_path = os.path.join(root, file)
                clean_notebook_in_place(full_path)

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print("Usage: python remove_emojis_in_place.py /path/to/folder")
    else:
        process_all_notebooks_in_folder(sys.argv[1])

