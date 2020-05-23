from math import sqrt

puntos = [
    [6, 10, "base station Leader"], 
    [10, 2, "base station Follower0"],
    [5, 9, "objective Leader"],
    [16, 13, "objective Follower0"],
    [7, 7, "init position Leader"],
    [7, 2, "init position Follower0"]
]

for i in range(len(puntos)):
    for j in range(i, len(puntos)):
        v = sqrt( (puntos[j][0] - puntos[i][0])**2 + (puntos[j][1] - puntos[i][1])**2)
        print(puntos[i][2], "-", puntos[j][2], "=", round(v, 2))
    print()
