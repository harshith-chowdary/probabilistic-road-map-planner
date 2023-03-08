from scipy.spatial.distance import cdist

def closest_nodes(k,node,nodes):
    cnodes=[]
    while len(cnodes)<k:
        cnode=nodes[cdist([node], nodes).argmin()]
        cnodes.append(cnode)
        nodes.remove(cnode)
    return cnodes

print(closest_nodes(5,[1,1],[[1,2],[2,2],[-1,0],[-2,1],[3,0]]))