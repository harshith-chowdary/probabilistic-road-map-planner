from matplotlib import path
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

obs1=[[5,11],[14,5],[11,20]]
obs2=[[12,29],[15,31],[15,34],[12,34]]
obs3=[[25,9],[25,20],[32,15],[35,6]]
obs4=[[53,32],[39,32],[36,27],[42,19],[53,24]]

polygon=Polygon(obs1)   
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
x, y = polygon.exterior.xy
plt.plot(x, y, c="blue")

p=path.Path(obs1)
print(p.contains_point([5,11]))

