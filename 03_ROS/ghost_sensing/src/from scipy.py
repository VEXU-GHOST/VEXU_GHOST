from scipy.stats import gamma

shape_param = 44
scale_param = 1 / 0.7
time = 60

probability = gamma.cdf(time, a=shape_param, scale=scale_param)
print(f"P(X ≤ 60) ≈ {probability:.4f}")
