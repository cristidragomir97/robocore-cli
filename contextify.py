#!/usr/bin/env python3

import os
import argparse

try:
    import pyperclip
    CLIPBOARD_AVAILABLE = True
except ImportError:
    CLIPBOARD_AVAILABLE = False

def gather_source_files(folder, extensions=None):
    if extensions is None:
        extensions = {'.py', '.cpp', '.h', '.c', '.hpp', '.java', '.js', '.ts', '.html', '.css'}
    source_files = []
    for root, _, files in os.walk(folder):
        for file in files:
            if any(file.endswith(ext) for ext in extensions):
                source_files.append(os.path.join(root, file))
    return source_files

def contextify(folder, extensions=None):
    files = gather_source_files(folder, extensions)
    combined_content = ""
    for file_path in sorted(files):
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            relative_path = os.path.relpath(file_path, folder)
            combined_content += f"\n\n# --- Begin file: {relative_path} ---\n"
            combined_content += f.read()
            combined_content += f"\n# --- End file: {relative_path} ---\n"
    return combined_content

def main():
    parser = argparse.ArgumentParser(description="Concatenate source files in a folder into one string.")
    parser.add_argument("folder", help="Path to the folder to scan for source files.")
    parser.add_argument("--ext", nargs='*', default=None,
                        help="Optional list of file extensions to include (e.g. --ext .py .cpp)")
    parser.add_argument("--copy", action="store_true",
                        help="Copy the resulting string to the clipboard (requires pyperclip).")
    args = parser.parse_args()

    result = contextify(args.folder, set(args.ext) if args.ext else None)


    if args.copy:
        if CLIPBOARD_AVAILABLE:
            pyperclip.copy(result)
            print("\n✅ Copied to clipboard.")
        else:
            print("\n⚠️ pyperclip not installed. Run 'pip install pyperclip' to enable clipboard support.")

if __name__ == "__main__":
    main()
