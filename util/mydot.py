def mydot(vec1,vec2):
    dotprod = 0
    for i in range(0,len(vec1)):
        dotprod = dotprod + vec1[i]*vec2[i]
    return dotprod