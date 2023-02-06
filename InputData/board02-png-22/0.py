with open("list.txt","w") as f:
    for i in range(1,23):
        if i < 10:
            f.write("0%d.png\n" % i)
        else:
            f.write("%d.png\n" % i)