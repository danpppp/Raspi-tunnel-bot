import statistics as stats
list1=[1,2,3]
print(stats.mean(list1))
list1.append(4)
print(stats.mean(list1))
list1.clear()
print(list1)
list1.append(1)
print(list1)