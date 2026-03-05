"""Compute 2x2 covariance matrix of (x, y) for three pose datasets (hardcoded)."""

# no noise
NO_NOISE = [
    (0.3282296220089437, -0.03397134654890639),
    (0.23065114471249698, -0.31257667837181363),
    (-0.1423819274510923, 0.2875149338204416),
    (0.3017724491856204, 0.11863950277419236),
    (-0.2549186639021187, -0.17650288439155042),
    (0.09248177533910452, 0.34218845011377295),
    (0.2597448836609112, -0.14733899055120366),
    (-0.08761933244105827, 0.2149037756629914),
    (0.1763095538917426, -0.2984177260042815),
]

# 0.001 noise
NOISE_001 = [
    (0.24052584126453785, -0.09325001898538617),
    (0.44843724108536553, -0.321021664867534),
    (0.18234152948120317, -0.04192855723190462),
    (0.36597241827490544, -0.19843755210473821),
    (0.51284390756231876, -0.40219384756210453),
    (0.30157482093475128, -0.28741520394817266),
    (0.12938475209183744, 0.06349218476293458),
    (-0.08427531894726153, 0.11846372984501732),
    (-0.19284736520473162, -0.05739218465027394),
    (0.07845193276419538, -0.01483726594017362),
]

# 0.3 noise
NOISE_03 = [
    (4.379248750796529, -1.1821700363137502),
    (1.8457479796696021, 4.107478544432612),
    (-1.0896278642363695, -1.2847642346852652),
    (2.713482195438721, -3.904128557219384),
    (-3.562194871234908, 1.276548903472615),
    (0.9541837265491284, 3.487291650823741),
    (-4.128573690214587, -0.7643921587342195),
    (3.041276548192305, 2.118374650928471),
    (-2.487315094618273, -3.156742083941762),
    (1.632748590123847, -0.9483716527341983),
]

SECTIONS = [
    ("no noise", NO_NOISE),
    ("0.001 noise", NOISE_001),
    ("0.3 noise", NOISE_03),
]


def covariance_matrix(points):
    """Compute 2x2 sample covariance matrix for (x, y).
    Returns [[var(x), cov(x,y)], [cov(y,x), var(y)]], denominator n-1.
    """
    n = len(points)
    if n < 2:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    var_x = sum((x - x_mean) ** 2 for x in xs) / (n - 1)
    var_y = sum((y - y_mean) ** 2 for y in ys) / (n - 1)
    cov_xy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / (n - 1)
    return [[var_x, cov_xy], [cov_xy, var_y]]


def main():
    for name, points in SECTIONS:
        cov_mat = covariance_matrix(points)
        x_mean = sum(p[0] for p in points) / len(points)
        y_mean = sum(p[1] for p in points) / len(points)
        print(f"--- {name} ---")
        print(f"  Number of points: {len(points)}")
        print(f"  x mean: {x_mean},  y mean: {y_mean}")
        print("  Covariance matrix (2x2):")
        print("    [ var(x)   cov(x,y) ]   [ {:>12.6f}  {:>12.6f} ]".format(cov_mat[0][0], cov_mat[0][1]))
        print("    [ cov(y,x)  var(y)  ] = [ {:>12.6f}  {:>12.6f} ]".format(cov_mat[1][0], cov_mat[1][1]))
        print()


if __name__ == "__main__":
    main()
