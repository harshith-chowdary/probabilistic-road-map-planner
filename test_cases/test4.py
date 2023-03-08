import shapely.geometry

def valid_dir_link(p1,p2):
    line_seg=shapely.geometry.LineString([p1,p2])

    obs1=[[5,11],[14,5],[11,20]]
    obs2=[[12,29],[15,31],[15,34],[12,34]]
    obs3=[[25,9],[25,20],[32,15],[35,6]]
    obs4=[[53,32],[39,32],[36,27],[42,19],[53,24]]

    poly1=shapely.geometry.Polygon(obs1)
    poly2=shapely.geometry.Polygon(obs2)
    poly3=shapely.geometry.Polygon(obs3)
    poly4=shapely.geometry.Polygon(obs4)

    if line_seg.intersects(poly1):
        return False
    elif line_seg.intersects(poly2):
        return False
    elif line_seg.intersects(poly3):
        return False
    elif line_seg.intersects(poly4):
        return False
    else:
        return True

print(valid_dir_link([2,10],[15,10]))