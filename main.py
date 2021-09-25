from RoutePlanner import RoutePlanner
from Helpers import load_map_40

if __name__ == '__main__':
    planner = RoutePlanner(load_map_40(), 5, 34)
    print(planner.path, "RESULT PATH")
    path = planner.path
    if path == [5, 16, 37, 12, 34]:
        print("great! Your code works for these inputs!")
    else:
        print("something is off, your code produced the following:")
        print(path)
