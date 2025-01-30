def create_folder(folder):
    import os, shutil
    try:
        os.mkdir(folder)
    except FileExistsError:
        shutil.rmtree(folder)
        os.mkdir(folder)

def get_last_path_segment(path: str) -> str:
    """Returns the last segment of a given path or URL after the last slash."""
    return path.rstrip("/").rsplit("/", 1)[-1]

