from RoutePlanner import RoutePlanner
from Helpers import Map, load_map_10, load_map_40, show_map

if __name__ == '__main__':
    # show_map(load_map_40(), 5, 34)
    planner = RoutePlanner(load_map_40(), 5, 34)
    path = planner.path
    if path == [5, 16, 37, 12, 34]:
        print("great! Your code works for these inputs!")
    else:
        print("something is off, your code produced the following:")
        print(path)
