from functions.functions import *
import matplotlib.pyplot as plt
import time
#from dissertation.space_conf import obstaclelist,circles
ax = plt.gca()
ax.cla()
ax.set_xlim((-20, 120))
ax.set_ylim((-20, 120))

n_vertices = 20 # number of vertices of the graph
k_near = 20         # number of nearest nodes
stepx = stepy = 3 #enter steps distance
start = [node(0,0), node(100,0)] # enter start node if multiple drones enter as start = [node(x1,y1), node(x2,y2, ...]
goal = [node(100,100), node(0,100)]  # enter goal node if multiple drones enter as goal = [node(x1,y1), node(x2,y2, ...]
safesize = 5     # enter drone safesize

minRand = 0             # enter minimum random limit
maxRand = 100           # enter maximum random limit
# -------OBSTACLE DECLARATION_------------#
#obstaclelist = [[node(x1,y1), radius1], [node(x2,y2), radius2], ....]
#obstaclelist = []
obstaclelist = [[node(50,50), 10], [node(30,60), 10], [node(60,30), 10], [node(60,60), 10], [node(60,40), 10], [node(40,70), 10], [node(60,80), 10]]
# ----------------------------------------#
separation_of_pathes = False  #enter true if want to make pathes generation without interception





circles = []
for i in range(len(obstaclelist)):
    circles.append(plt.Circle((obstaclelist[i][0].x, obstaclelist[i][0].y), obstaclelist[i][1]))
for i in range(len(circles)):
    ax.add_artist(circles[i])
drones = []
for i in range(len(start)):
    drones.append(drone(start = start[i], goal =goal [i], safesize = safesize, obstaclelist = obstaclelist))

start_time = time.time()
list = random_node_list_generation(n_vertices,minRand,maxRand, obstaclelist,safesize = safesize)
for i in range(len(drones)):
    list.insert(i,drones[i].start)
    list.insert(len(list) - i, drones[i].goal)
for i in range(len(list)):
    times = time.time()
    nearest = k_nearest(nodelist =list, k = k_near, obstaclelist = obstaclelist, safesize = safesize, node = list[i])
    list[i].nearest = nearest
graph = initialize_graph(list)
learning_phase_time = time.time()
learning_phase_duration =  time.time() - start_time
for i in range(len(drones)):
    drones[i].path = dijsktra(graph = graph, initial = list[i], end = list[len(list) - 1 - i])
query_time =  time.time() - learning_phase_time

b = separation_of_pathes
while b == True:
    for i in range(len(drones)):
        if i == len(drones) - 1:
            a = 0
            params = path_intercepting(path1 = drones[i].path, path2 = drones[a].path, safesize1 =safesize , safesize2 = safesize, stepx = stepx, stepy = stepy)
        else:
            a = a + 1
            params = path_intercepting(path1=drones[i].path, path2=drones[a].path, safesize1=safesize,
                                       safesize2=safesize, stepx=stepx, stepy=stepy)
        if params [0] == True:
            b = False
        else:

            remove1 = list.index(params[0][0])
            remove2 = list.index(params[0][1])
            remove1 = list[remove1]
            remove2 = list[remove2]
            graph.remove_edge(from_node=remove1, to_node=remove2)

            remove1 = list.index(params[1][0])
            remove2 = list.index(params[1][1])
            remove1 = list[remove1]
            remove2 = list[remove2]

            graph.remove_edge(from_node=remove1, to_node = remove2)
            drones[i].path = dijsktra(graph, initial = list[i], end = list[len(list) - 1 - i])
            drones[a].path = dijsktra(graph, initial=list[1], end=list[len(list) - 1 - a])


for i in range(len(list)):
    plt.plot(list[i].x, list[i].y, marker = 'x', color = 'g')

x = [[] for i in range(len(drones))]
y = [[] for i in range(len(drones)) ]

for k in range(len(drones)):
    distance = 0
    for i in range(len(drones[k].path)):
        x[k].append(drones[k].path[i].x)
        y[k].append(drones[k].path[i].y)
        if i < len(drones[k].path) - 1:
            distance = distance + node_distance(node1 = drones[k].path[i], node2 = drones[k].path[i + 1])
    drones[k].distance = distance


path_format = [[] for i in range(len(drones))]
for j in range(len(drones)):
    for i in range(len(drones[j].path)):
        path_format[j].append([drones[j].path[i].x, drones[j].path[i].y])


objects = [[] for i in range(len(drones))]
colors = ['r', 'y', 'g']
for j in range(len(drones)):
    for i in range(len(x)):
        objects[j].append(plt.Circle((x[j][i], y[j][i]), safesize))
for j in range(len(drones)):
    for i in range(len(objects[j])):
        ax.add_artist(objects[j][i])
        objects[j][i].set_edgecolor(colors[j])
        objects[j][i].set_facecolor('none')
for i in range(len(drones)):
    plt.plot(x[i],y[i], color = colors[i])
    plt.plot(x[i], y[i], color = colors[i])

#-------------OUTPUT--------------#
for i in range(len(drones)):
    print("-----Drone Number-> ", i + 1, " ------")
    print("Path generated for this drone-> ", path_format[i])
    print("Path length -> ",drones[i].distance)

print('Learning phase duration ->' ,learning_phase_duration)
print('Query phase duration ->', query_time)


plt.show(block=False)
plt.pause(4)
plt.close('all')



