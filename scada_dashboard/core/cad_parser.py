import numpy as np
from PIL import Image, ImageFilter, ImageOps

DEFAULT_GRID = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

def parse_cad_image(uploaded_file, grid_shape=(20, 12)):
    img = Image.open(uploaded_file).convert("L")
    img = ImageOps.autocontrast(img, cutoff=2)
    img_thick = img.filter(ImageFilter.MinFilter(3))
    img_resized = img_thick.resize(grid_shape, Image.NEAREST)
    img_arr = np.array(img_resized)
    grid = (img_arr < 190).astype(int).tolist()
    
    h, w = len(grid), len(grid[0])
    for c in range(w):
        grid[0][c] = 0
        grid[h-1][c] = 0
        grid[h//2][c] = 0
    for r in range(h):
        grid[r][0] = 0
        grid[r][w-1] = 0
        grid[r][w//2] = 0
        
    return grid